---
id: bd-twins
status: active
updated: 2026-11-09
stopped_at: Atlas v0 em MJCF, check passa
next: medir proporções nas fotos do Atlas
tasks:
  - Medir segmentos nas fotos/vídeo
  - Atlas com rodas (tornozelo contínuo)
  - Plugar no mjlab (constants + atuadores)
  - Modelo do Stretch
---

# bd-twins

Modelos de simulação (MuJoCo/mjlab) das plataformas da Boston Dynamics que não têm modelo público: Atlas elétrico e Stretch. Depois: sistema de frota com Atlas + Spot + Stretch na mesma missão.

Por quê: `~/second-brain/05-build/ideas/bd-showcase-project.md`.

## Onde parei
Atlas v0 do zero: 28 DoF de corpo, proporções humanas escaladas pro spec sheet (90 kg, 1,9 m). Fontes em `docs/atlas-sources.md`. `python atlas/check.py` passa.

## Histórico
- 2026-11-09 · Repo criado. Atlas v0 (MJCF, sem atuadores) + check contra o spec sheet.
