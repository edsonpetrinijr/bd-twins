---
id: bd-twins
status: active
updated: 2026-11-09
stopped_at: v1.0, corpo quase todo em CAD
next: Mãos, pés e ombros em CAD
tasks:
  - Casca visual mais fiel (mesh)
  - Treinar Flat-BD-Atlas na GPU
  - Modelo do Stretch
---

# bd-twins

Modelos de simulação (MuJoCo/mjlab) das plataformas da Boston Dynamics que não têm modelo público: Atlas elétrico e Stretch. Depois: sistema de frota com Atlas + Spot + Stretch na mesma missão.

Por quê: `~/second-brain/05-build/ideas/bd-showcase-project.md`.

## Onde parei
Atlas v0.1 do zero: 28 DoF de corpo. Pernas e braços medidos nas fotos do spec sheet; massas e larguras ainda estimadas. Fontes em `docs/atlas-sources.md`. `python atlas/check.py` passa.

## Histórico
- 2026-11-09 · Repo criado. Atlas v0 (MJCF, sem atuadores) + check contra o spec sheet.
- 2026-11-09 · v0.1: proporções medidas nas fotos (tornozelo 0,196 m, canela 0,336, coxa 0,467, ombro 1,58, braço 0,28/0,28/0,16).
- 2026-11-09 · v0.2: casca visual (formas arredondadas, cores, cabeça com anel, mochila, mão de 3 dedos). `atlas/build.py` gera o XML; `atlas/render.py` compara com as fotos. Versão com rodas descartada pelo Edson.
- 2026-11-09 · v0.3: `atlas/compare.py` compara com a máscara oficial do render (IoU 0.796 → 0.837 em 4 rodadas). Maior achado: ombro mais baixo (1,51 m) e braço mais longo.
- 2026-11-09 · v0.4: ombro, tronco, pelve e pernas refeitos com fotos do site da BD + frames do vídeo de produto. `atlas/views.py` compara 5 vistas lado a lado.
- 2026-11-09 · v0.5/v0.6: massa por corpo (estimativa documentada) com inércia das cascas; atuadores estimados; tarefas `Mjlab-Velocity-{Flat,Rough}-BD-Atlas`. Teste no Mac: 200 passos sem NaN nem queda.
- 2026-11-09 · v0.7/v0.8: perna refeita (canela em placa, juntas em tambor) e ombro/tronco/quadril mais estreitos a pedido do Edson.
- 2026-11-09 · v0.9: primeiras peças em CAD (build123d): canela e tambores de joelho/tornozelo. `atlas/cad.py` gera os STL.
- 2026-11-09 · v0.10/v1.0: coxa, braço, antebraço, cotovelo, tronco (barriga, domo, cintura), tambores frisados da pelve e cabeça (casca, anel, face rebaixada) viraram CAD.
