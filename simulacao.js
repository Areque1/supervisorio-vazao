/* ===== Núcleo da simulação: planta + PID + alarmes + registro ===== */
const Sim = (() => {
  // Planta: tanque -> bomba centrífuga B-101 -> PT-101 -> FT-101 -> FCV-101 -> PT-102 -> destino
  const PLANT = {
    Psuct: 0.5,    // bar, pressão de sucção (coluna do tanque)
    H0: 6.0,       // bar, pressão diferencial de shutoff da bomba a 100% de rotação
    Pback: 0.5,    // bar, contrapressão no destino
    kp: 2.5e-4,    // bar/(L/min)^2, queda interna da curva da bomba
    kl: 1.2e-4,    // bar/(L/min)^2, perda de carga da linha (jusante da válvula)
    CvMax: 109,    // (L/min)/sqrt(bar), válvula 100% aberta
    R: 25,         // rangeabilidade (característica igual-percentagem)
    kObs: 1.2e-2,  // obstrução (filtro Y entupido)
    CvLeak: 60,    // vazamento de grande porte entre FT-101 e FCV-101
    tauQ: 0.9,     // s, constante de tempo hidráulica
    deadT: 0.4,    // s, tempo morto (transporte)
    tauV: 0.5,     // s, posicionador da válvula
    rateV: 40,     // %/s, velocidade máxima de curso
    noise: 0.35,   // L/min, ruído de turbulência (desvio padrão)
    rampUp: 3.0,   // s, partida suave (soft-starter)
    rampDown: 1.5  // s, parada
  };
  const DT = 0.1;          // s, passo de integração e varredura do CLP
  const LOG_EVERY = 0.5;   // s, período de registro histórico
  const LOG_MAX = 120000;  // ~16 h de histórico a 0,5 s

  const ALARM_DEFS = [
    { tag: 'FAHH-101', desc: 'Vazão muito alta', prio: 'Alta', kind: 'flow', test: (s, c) => s.pv > c.alm.HH, clear: (s, c) => s.pv < c.alm.HH - 1.5 },
    { tag: 'FAH-101', desc: 'Vazão alta', prio: 'Média', kind: 'flow', test: (s, c) => s.pv > c.alm.H, clear: (s, c) => s.pv < c.alm.H - 1.5 },
    { tag: 'FAL-101', desc: 'Vazão baixa', prio: 'Média', kind: 'flow', test: (s, c) => s.runFor > 8 && s.pv < c.alm.L, clear: (s, c) => s.runFor <= 8 || s.pv > c.alm.L + 1.5 },
    { tag: 'FALL-101', desc: 'Vazão muito baixa', prio: 'Alta', kind: 'flow', test: (s, c) => s.runFor > 8 && s.pv < c.alm.LL, clear: (s, c) => s.runFor <= 8 || s.pv > c.alm.LL + 1.5 },
    { tag: 'XA-101', desc: 'Falha de sinal FT-101 (< 3,8 mA)', prio: 'Alta', kind: 'sig', test: (s) => s.mA < 3.8, clear: (s) => s.mA >= 3.8 },
    { tag: 'PAH-101', desc: 'Pressão de descarga alta', prio: 'Alta', kind: 'press', test: (s, c) => s.p1 > c.alm.PAH, clear: (s, c) => s.p1 < c.alm.PAH - 0.1 },
    { tag: 'PAL-101', desc: 'Pressão de descarga baixa', prio: 'Média', kind: 'press', test: (s, c) => s.runFor > 8 && s.p1 < c.alm.PAL, clear: (s, c) => s.runFor <= 8 || s.p1 > c.alm.PAL + 0.1 },
    { tag: 'YA-101', desc: 'Desarme da bomba B-101 (relé térmico)', prio: 'Alta', kind: 'equip', test: (s) => s.faults.trip, clear: (s) => !s.faults.trip }
  ];
  const ALARM_DELAY = 1.0; // s, retardo de atuação (anti-chattering)

  let seed = 12345;
  function rand() { // mulberry32
    seed = (seed + 0x6D2B79F5) | 0; let t = seed;
    t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }
  function randn() { let u = 0, v = 0; while (u === 0) u = rand(); v = rand(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  const clamp = (x, a, b) => Math.min(b, Math.max(a, x));

  function cvOf(u) { return u <= 0.5 ? 0 : PLANT.CvMax * Math.pow(PLANT.R, u / 100 - 1); }

  function create() {
    seed = 12345;
    return {
      t: 0,
      cfg: {
        Kp: 0.6, Ti: 1.2, Td: 0.0, outMin: 10, outMax: 90, range: 150,
        alm: { HH: 105, H: 85, L: 25, LL: 10, PAH: 6.3, PAL: 2.0 }
      },
      sp: 50, mode: 'AUTO', manOut: 40, out: 10,
      pid: { I: 10, pvPrev: 0, D: 0 },
      pumpCmd: false, pumpSpeed: 0, runFor: 0,
      valvePos: 0, valveDir: 0, valveLastExt: 0,
      q: 0, qBuf: [], nz: 0,
      qm: 0, mA: 4, pv: 0, bad: false, p1: 0.5, p2: 0.5,
      total: 0, totalSince: 0,
      pumpHours: 0, pumpStarts: 0, valveTravel: 0, valveCycles: 0,
      faults: { leak: false, obstruction: false, cable: false, trip: false, cavitation: false },
      alarms: Object.fromEntries(ALARM_DEFS.map(d => [d.tag, { state: 'NORMAL', timer: 0, since: null }])),
      alarmHist: [], events: [], log: [], logAcc: 0, logStart: 0
    };
  }

  function event(s, text, kind = 'op') { s.events.push({ t: s.t, text, kind }); if (s.events.length > 3000) s.events.shift(); }

  function steady(s) {
    const f = s.faults, n = s.pumpSpeed;
    const head = PLANT.H0 * n * n * (f.cavitation ? 0.3 : 1);
    const psuct = f.cavitation ? -0.3 : PLANT.Psuct;
    const dP = psuct + head - PLANT.Pback;
    const cvV = cvOf(s.valvePos);
    const cv = cvV + (f.leak ? PLANT.CvLeak : 0);
    if (dP <= 0 || cv <= 0) return { Q: 0, psuct, head, frac: 1 };
    const res = PLANT.kp + PLANT.kl + (f.obstruction ? PLANT.kObs : 0) + 1 / (cv * cv);
    return { Q: Math.sqrt(dP / res), psuct, head, frac: cvV / cv };
  }

  function pidStep(s, dt) {
    const c = s.cfg;
    const pvPct = s.pv / c.range * 100, spPct = s.sp / c.range * 100;
    const e = spPct - pvPct;
    const running = s.pumpCmd && s.pumpSpeed > 0.2;
    if (s.mode === 'AUTO' && s.bad) {
      s.mode = 'MAN'; s.manOut = s.out;
      event(s, 'Malha FIC-101 forçada para MANUAL: sinal do FT-101 inválido. Saída mantida em ' + s.out.toFixed(1) + '%', 'sys');
    }
    if (s.mode === 'MAN') {
      s.out = clamp(s.manOut, 0, 100);
      s.pid.I = s.out - c.Kp * e; s.pid.D = 0; s.pid.pvPrev = pvPct; // bumpless
      return;
    }
    if (!running) { // AUTO com bomba parada: saída em rastreamento no limite mínimo
      s.out = c.outMin; s.pid.I = s.out - c.Kp * e; s.pid.D = 0; s.pid.pvPrev = pvPct; return;
    }
    // Derivada sobre a PV com filtro de 1ª ordem (N = 8)
    if (c.Td > 0) {
      const Tf = c.Td / 8;
      s.pid.D = (Tf / (Tf + dt)) * s.pid.D - (c.Kp * c.Td / (Tf + dt)) * (pvPct - s.pid.pvPrev);
    } else s.pid.D = 0;
    s.pid.pvPrev = pvPct;
    const P = c.Kp * e;
    const dI = c.Ti > 0 ? c.Kp * dt / c.Ti * e : 0;
    const uTry = P + s.pid.I + dI + s.pid.D;
    // Anti-windup por integração condicional
    if ((uTry < c.outMax || dI < 0) && (uTry > c.outMin || dI > 0)) s.pid.I += dI;
    s.out = clamp(P + s.pid.I + s.pid.D, c.outMin, c.outMax);
  }

  function evalAlarms(s, dt) {
    for (const d of ALARM_DEFS) {
      const a = s.alarms[d.tag];
      const active = a.state === 'ATIVO_NR' || a.state === 'ATIVO_R';
      if (!active) {
        if (d.test(s, s.cfg)) { a.timer += dt; if (a.timer >= ALARM_DELAY) {
          a.state = 'ATIVO_NR'; a.since = s.t; a.timer = 0;
          s.alarmHist.push({ t: s.t, tag: d.tag, desc: d.desc, prio: d.prio, what: 'Ativado', val: alarmValue(s, d) });
        } } else a.timer = 0;
      } else if (d.clear(s, s.cfg)) {
        a.state = a.state === 'ATIVO_R' ? 'NORMAL' : 'RTN_NR';
        s.alarmHist.push({ t: s.t, tag: d.tag, desc: d.desc, prio: d.prio, what: 'Normalizado', val: alarmValue(s, d) });
      }
    }
  }
  function alarmValue(s, d) {
    if (d.kind === 'flow') return s.pv.toFixed(1) + ' L/min';
    if (d.kind === 'press') return s.p1.toFixed(2) + ' bar';
    if (d.kind === 'sig') return s.mA.toFixed(2) + ' mA';
    return '—';
  }
  function ack(s, tag) {
    const a = s.alarms[tag]; if (!a) return;
    if (a.state === 'ATIVO_NR') a.state = 'ATIVO_R';
    else if (a.state === 'RTN_NR') a.state = 'NORMAL';
    else return;
    const d = ALARM_DEFS.find(x => x.tag === tag);
    s.alarmHist.push({ t: s.t, tag, desc: d.desc, prio: d.prio, what: 'Reconhecido', val: '' });
  }

  function step(s) {
    const dt = DT;
    s.t += dt;
    // Bomba
    const target = s.pumpCmd && !s.faults.trip ? 1 : 0;
    if (s.faults.trip && s.pumpCmd) { s.pumpCmd = false; event(s, 'Bomba B-101 desarmada (proteção térmica atuou)', 'sys'); }
    s.pumpSpeed = target >= s.pumpSpeed ? Math.min(target, s.pumpSpeed + dt / PLANT.rampUp) : Math.max(target, s.pumpSpeed - dt / PLANT.rampDown);
    s.runFor = s.pumpCmd ? s.runFor + dt : 0;
    if (s.pumpSpeed > 0.05) s.pumpHours += dt;

    // Controlador (CLP) usa a PV da varredura anterior
    pidStep(s, dt);

    // Válvula: posicionador de 1ª ordem com limite de velocidade
    const prev = s.valvePos;
    const dv = clamp((s.out - s.valvePos) * dt / PLANT.tauV, -PLANT.rateV * dt, PLANT.rateV * dt);
    s.valvePos = clamp(s.valvePos + dv, 0, 100);
    const move = s.valvePos - prev;
    s.valveTravel += Math.abs(move);
    const dir = move > 0.02 ? 1 : move < -0.02 ? -1 : 0;
    if (dir !== 0) {
      if (s.valveDir !== 0 && dir !== s.valveDir && Math.abs(s.valvePos - s.valveLastExt) > 0.5) s.valveCycles += 0.5;
      if (dir !== s.valveDir) s.valveLastExt = prev;
      s.valveDir = dir;
    }

    // Hidráulica: tempo morto + 1ª ordem
    const st = steady(s);
    s.qBuf.push(st.Q);
    const nDelay = Math.round(PLANT.deadT / dt);
    const qd = s.qBuf.length > nDelay ? s.qBuf.shift() : 0;
    s.q += (qd - s.q) * dt / PLANT.tauQ;
    if (s.q < 0) s.q = 0;

    // Pressões
    s.p1 = st.psuct + st.head - PLANT.kp * s.q * s.q + randn() * 0.01;
    const qv = s.q * st.frac;
    s.p2 = (s.pumpSpeed > 0 || s.q > 0.1 ? PLANT.Pback + PLANT.kl * qv * qv : PLANT.Pback) + randn() * 0.005;

    // Transmissor FT-101 (4–20 mA) com ruído de turbulência
    const sig = PLANT.noise * (s.faults.cavitation ? 5 : 1);
    s.nz = 0.6 * s.nz + 0.4 * randn() * sig * 1.6;
    s.qm = s.q > 0.05 ? s.q + s.nz : Math.max(0, s.nz * 0.1);
    const c = s.cfg;
    s.mA = s.faults.cable ? 0.02 + Math.abs(randn()) * 0.01 : clamp(4 + 16 * s.qm / c.range, 3.9, 20.5);
    s.bad = s.mA < 3.8 || s.mA > 20.6;
    if (!s.bad) s.pv = Math.max(0, (s.mA - 4) / 16 * c.range);

    // Totalizador (usa PV válida; congela em falha de sinal)
    if (!s.bad) { const v = s.pv * dt / 60; s.total += v; }

    evalAlarms(s, dt);

    s.logAcc += dt;
    if (s.logAcc >= LOG_EVERY - 1e-9) {
      s.logAcc = 0;
      s.log.push({ t: +s.t.toFixed(1), sp: s.sp, pv: s.bad ? null : s.pv, out: s.out, vp: s.valvePos, p1: s.p1, p2: s.p2, mA: s.mA, tot: s.total,
        mode: s.mode, pump: s.pumpSpeed > 0.05 ? 1 : 0, flt: Object.values(s.faults).some(Boolean) ? 1 : 0 });
      if (s.log.length > LOG_MAX) s.log.shift();
    }
  }

  // ---- Comandos do operador ----
  const cmd = {
    setSP(s, v) { v = clamp(v, 0, s.cfg.range); if (Math.abs(v - s.sp) < 1e-9) return; event(s, `Setpoint alterado: ${s.sp.toFixed(1)} → ${v.toFixed(1)} L/min`); s.sp = v; },
    setMode(s, m) { if (m === s.mode) return; if (m === 'AUTO' && s.bad) { event(s, 'Tentativa de AUTO recusada: sinal do FT-101 inválido', 'sys'); return false; }
      if (m === 'MAN') s.manOut = s.out; s.mode = m; event(s, `Modo da malha: ${m === 'AUTO' ? 'AUTOMÁTICO' : 'MANUAL'}`); return true; },
    setManOut(s, v) { v = clamp(v, 0, 100); event(s, `Saída manual: ${s.manOut.toFixed(1)} → ${v.toFixed(1)}%`); s.manOut = v; },
    pump(s, on) { if (on === s.pumpCmd) return true; if (on && s.faults.trip) { event(s, 'Partida da bomba bloqueada: relé térmico desarmado', 'sys'); return false; }
      s.pumpCmd = on; if (on) s.pumpStarts++; event(s, on ? 'Bomba B-101 ligada' : 'Bomba B-101 desligada'); return true; },
    resetTotal(s) { event(s, `Totalizador zerado (valor anterior: ${s.total.toFixed(1)} L)`); s.total = 0; s.totalSince = s.t; },
    fault(s, name, on) { const names = { leak: 'Vazamento a jusante do FT-101', obstruction: 'Obstrução na linha (filtro entupido)', cable: 'Rompimento do cabo 4–20 mA do FT-101', trip: 'Desarme térmico da bomba', cavitation: 'Baixa pressão de sucção (cavitação)' };
      if (s.faults[name] === on) return; s.faults[name] = on; event(s, `${on ? 'Falha simulada INSERIDA' : 'Falha simulada REMOVIDA'}: ${names[name]}`, 'fault'); },
    tune(s, p) { const c = s.cfg, ch = [];
      for (const k of ['Kp', 'Ti', 'Td', 'outMin', 'outMax']) if (p[k] !== undefined && p[k] !== c[k]) { ch.push(`${k} ${c[k]} → ${p[k]}`); c[k] = p[k]; }
      for (const k of Object.keys(p.alm || {})) if (p.alm[k] !== c.alm[k]) { ch.push(`${k} ${c.alm[k]} → ${p.alm[k]}`); c.alm[k] = p.alm[k]; }
      if (ch.length) event(s, 'Parâmetros de engenharia alterados: ' + ch.join('; ')); return ch.length; },
    ack, ackAll(s) { for (const tag in s.alarms) ack(s, tag); }
  };

  // ---- Análise para relatório ----
  function stats(arr) {
    const v = arr.filter(x => x !== null && isFinite(x)); if (!v.length) return null;
    let sum = 0, min = Infinity, max = -Infinity; for (const x of v) { sum += x; if (x < min) min = x; if (x > max) max = x; }
    const mean = sum / v.length; let sq = 0; for (const x of v) sq += (x - mean) ** 2;
    return { mean, min, max, std: Math.sqrt(sq / v.length), n: v.length };
  }

  // Analisa cada degrau de setpoint em AUTO: tempo de subida 10–90%, sobressinal, acomodação ±2%, IAE
  function stepAnalysis(log) {
    const out = [];
    for (let i = 1; i < log.length; i++) {
      if (log[i].sp === log[i - 1].sp) continue;
      const sp0 = log[i - 1].sp, sp1 = log[i].sp, t0 = log[i].t, d = sp1 - sp0;
      if (Math.abs(d) < 2) continue;
      let j = i; while (j + 1 < log.length && log[j + 1].sp === sp1) j++;
      // janela de análise: até 120 s, interrompida se a malha sair de AUTO, a bomba parar, o sinal falhar ou uma falha simulada for inserida
      const ok = r => r.mode === 'AUTO' && r.pump === 1 && r.pv !== null && !r.flt;
      let k = i; while (k <= j && ok(log[k]) && log[k].t - t0 <= 120) k++;
      const seg = log.slice(i, k);
      const pv0 = log[i - 1].pv;
      const row = { t0, sp0, sp1, dur: seg.length ? seg[seg.length - 1].t - t0 : 0, valid: ok(log[i - 1]) && pv0 !== null && seg.length >= 40 };
      if (row.valid) {
        // suavização leve (média móvel de 3) para reduzir efeito do ruído
        const pv = seg.map((r, k) => (seg[Math.max(0, k - 1)].pv + r.pv + seg[Math.min(seg.length - 1, k + 1)].pv) / 3);
        const sgn = Math.sign(d);
        const frac = pv.map(x => (x - pv0) / d);
        const k10 = frac.findIndex(f => f >= 0.1), k90 = frac.findIndex(f => f >= 0.9);
        row.rise = k10 >= 0 && k90 >= 0 ? seg[k90].t - seg[k10].t : null;
        const peak = sgn > 0 ? Math.max(...pv) : Math.min(...pv);
        row.overshoot = Math.max(0, (peak - sp1) * sgn / Math.abs(d) * 100);
        const band = Math.max(0.02 * Math.abs(sp1), 0.6);
        let last = -1; for (let k = 0; k < pv.length; k++) if (Math.abs(pv[k] - sp1) > band) last = k;
        row.settle = last === pv.length - 1 ? null : (last < 0 ? 0 : seg[last + 1].t - t0);
        let iae = 0; for (let k = 1; k < seg.length; k++) iae += Math.abs(sp1 - seg[k].pv) * (seg[k].t - seg[k - 1].t) / 60; // L
        row.iae = iae;
        row.essMean = stats(seg.slice(-Math.min(20, seg.length)).map(r => r.pv - sp1)).mean;
      }
      out.push(row);
    }
    return out;
  }

  function report(s) {
    const L = s.log; if (!L.length) return null;
    const auto = L.filter(r => r.mode === 'AUTO' && r.pump === 1 && r.pv !== null);
    const err = auto.map(r => r.pv - r.sp);
    const within = auto.filter(r => Math.abs(r.pv - r.sp) <= Math.max(0.02 * r.sp, 0.6)).length;
    const alarmCount = {};
    for (const h of s.alarmHist) if (h.what === 'Ativado') alarmCount[h.tag] = (alarmCount[h.tag] || 0) + 1;
    return {
      t0: L[0].t, t1: L[L.length - 1].t, n: L.length,
      pv: stats(L.map(r => r.pv)), sp: stats(L.map(r => r.sp)), out: stats(L.map(r => r.out)),
      p1: stats(L.filter(r => r.pump).map(r => r.p1)), p2: stats(L.filter(r => r.pump).map(r => r.p2)),
      err: stats(err), absErr: stats(err.map(Math.abs)), autoFrac: auto.length / L.length,
      inBand: auto.length ? within / auto.length : null,
      steps: stepAnalysis(L), alarmCount,
      total: s.total, pumpHours: s.pumpHours, pumpStarts: s.pumpStarts, valveCycles: s.valveCycles, valveTravel: s.valveTravel,
      cfg: JSON.parse(JSON.stringify(s.cfg))
    };
  }

  return { PLANT, DT, ALARM_DEFS, create, step, cmd, report, stats, cvOf };
})();
if (typeof module !== 'undefined') module.exports = Sim;
