# Supervisório FIC-101: controle de vazão simulado

IHM/SCADA simulada de uma malha de controle de vazão, com bomba centrífuga, válvula de controle, transmissores de vazão e pressão e um controlador PID. A página roda direto no navegador, sem instalação, e gera relatórios de operação com vazão, pressão, alarmes e desempenho da malha.

**Simulador online: https://areque1.github.io/supervisorio-vazao/**

Trabalho da disciplina **Análise e Processamento de Sinais** (2026).
Autores: Lucas Areque, Pedro Maciel, Samily Oliveira, Wildison Barbosa e Cauê Aguiar.

## Documentos

| Documento | Arquivo |
| --- | --- |
| Texto do trabalho | [`docs/trabalho.md`](docs/trabalho.md) |
| Apresentação (PDF, 12 slides) | [`docs/apresentacao.pdf`](docs/apresentacao.pdf) |

## Como usar

1. Acesse o [simulador online](https://areque1.github.io/supervisorio-vazao/) ou abra o `index.html` em qualquer navegador.
2. Na aba **Processo**, clique em **Executar ensaio automático**. Ele simula 12 minutos de operação em cerca de 13 segundos e abre o relatório no final.
3. Na aba **Relatório**, preencha a identificação e baixe o relatório (`.html`) ou os dados (`.csv`).

A aba **Engenharia (PID)** usa a senha `1234`.

## O que a simulação tem

| Item | Onde fica |
| --- | --- |
| PV em destaque, SP editável, CV (0–100 %), modo manual/automático | Aba Processo, painel FIC-101 |
| Vazão totalizada com reset, estado da bomba e da válvula, horímetro | Aba Processo |
| Sintonia Kp, Ti, Td e limites de saída | Aba Engenharia (com senha) |
| Alarmes HH/H/L/LL, pressão alta/baixa, falha de sinal abaixo de 3,8 mA | Aba Alarmes |
| Tendências de PV, SP, CV e pressões | Aba Tendências |
| Falhas simuladas: vazamento, obstrução, cavitação, cabo rompido, desarme da bomba | Painel do instrutor, aba Processo |
| Relatório com resposta ao degrau, estatísticas, alarmes e eventos | Aba Relatório |

## Estrutura

| Arquivo | Conteúdo |
| --- | --- |
| `index.html` | Interface (telas, gráficos, relatório) |
| `simulacao.js` | Modelo da planta, PID, alarmes e cálculo do relatório |
| `testes/ensaio.js` | Testes do modelo e do controlador |
| `docs/trabalho.md` | Texto do trabalho: modelo matemático, controle, IHM, alarmes e resultados |

## Testes

Com o Node.js instalado:

```
node testes/ensaio.js
```

Os testes verificam a regulação em 50 L/min, a resposta aos degraus de setpoint e se cada falha simulada dispara os alarmes esperados.
