---
id: bd-twins
status: active
updated: 2026-11-09
stopped_at: Atlas v0.1 com medidas das fotos
next: Atlas com rodas (tornozelo contínuo)
tasks:
  - Atlas com rodas (tornozelo contínuo)
  - Plugar no mjlab (constants + atuadores)
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
