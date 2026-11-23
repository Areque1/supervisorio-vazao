// Teste do modelo e do controlador sem interface gráfica.
// Uso: node testes/ensaio.js
const assert = require('node:assert/strict');
const Sim = require('../simulacao.js');

function rodar(ate, s, roteiro = {}) {
  const feito = new Set();
  while (s.t < ate) {
    const k = Math.floor(s.t + 1e-6);
    if (roteiro[k] && !feito.has(k)) { feito.add(k); roteiro[k](s); }
    Sim.step(s);
  }
}
const ativado = (s, tag) => s.alarmHist.some(h => h.tag === tag && h.what === 'Ativado');
let ok = 0;
function teste(nome, fn) { fn(); ok++; console.log('ok  ' + nome); }

teste('partida e regulação em 50 L/min', () => {
  const s = Sim.create(); Sim.cmd.pump(s, true); rodar(60, s);
  assert.ok(Math.abs(s.pv - 50) < 1.5, `PV = ${s.pv}`);
  assert.ok(s.p1 > 5.5 && s.p1 < 6.2, `PT-101 = ${s.p1}`);
});

teste('degraus de setpoint: sobressinal < 5 % e acomodação < 15 s', () => {
  const s = Sim.create(); Sim.cmd.pump(s, true);
  rodar(240, s, { 60: x => Sim.cmd.setSP(x, 80), 150: x => Sim.cmd.setSP(x, 30) });
  const degraus = Sim.report(s).steps.filter(d => d.valid);
  assert.equal(degraus.length, 2);
  for (const d of degraus) {
    assert.ok(d.overshoot < 5, `sobressinal ${d.overshoot}`);
    assert.ok(d.settle !== null && d.settle < 15, `acomodação ${d.settle}`);
  }
});

const falhas = [
  ['leak', ['FAH-101']],
  ['obstruction', ['FAL-101', 'PAH-101']],
  ['cavitation', ['PAL-101']],
  ['cable', ['XA-101']],
  ['trip', ['YA-101']]
];
for (const [falha, tags] of falhas) {
  teste(`falha "${falha}" dispara ${tags.join(' + ')}`, () => {
    const s = Sim.create(); Sim.cmd.pump(s, true);
    rodar(90, s, { 30: x => Sim.cmd.fault(x, falha, true) });
    for (const tag of tags) assert.ok(ativado(s, tag), `${tag} não ativou`);
  });
}

teste('cavitação em AUTO satura a saída sem alcançar o SP; em MAN a vazão cai abaixo de L', () => {
  const a = Sim.create(); Sim.cmd.pump(a, true);
  rodar(90, a, { 30: x => Sim.cmd.fault(x, 'cavitation', true) });
  assert.equal(a.out, a.cfg.outMax);
  assert.ok(a.pv < a.sp - 3, `PV = ${a.pv}`);
  const m = Sim.create(); Sim.cmd.pump(m, true);
  rodar(90, m, { 29: x => Sim.cmd.setMode(x, 'MAN'), 30: x => Sim.cmd.fault(x, 'cavitation', true) });
  assert.ok(ativado(m, 'FAL-101'), 'FAL-101 não ativou em MAN');
});

teste('falha de sinal força MANUAL e congela o totalizador', () => {
  const s = Sim.create(); Sim.cmd.pump(s, true);
  rodar(40, s, { 30: x => Sim.cmd.fault(x, 'cable', true) });
  assert.equal(s.mode, 'MAN');
  const total = s.total; rodar(60, s);
  assert.equal(s.total, total);
  assert.equal(Sim.cmd.setMode(s, 'AUTO'), false);
});

teste('sem ação integral (Ti = 0) sobra erro em regime', () => {
  const s = Sim.create(); s.cfg.Ti = 0; Sim.cmd.pump(s, true); rodar(60, s);
  assert.ok(Math.abs(s.sp - s.pv) > 5, `erro = ${s.sp - s.pv}`);
});

console.log(`\n${ok} testes passaram.`);
