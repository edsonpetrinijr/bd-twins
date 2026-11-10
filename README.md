# bd-twins

Boston Dynamics robots in MuJoCo / mjlab: electric Atlas (2026), Spot (`spot/`, sources in `docs/spot-sources.md`) and Stretch (`stretch/`, v0, sources in `docs/stretch-sources.md`). Electric Atlas, built only from public data (spec sheet, photos, videos). Every number and its source: `docs/atlas-sources.md`.

Personal project, not affiliated with or endorsed by Boston Dynamics. "Atlas" is a Boston Dynamics trademark, used here only to name the robot being modeled. Geometry, masses and actuator numbers are my own estimates from public material; no Boston Dynamics CAD, images or software are included.

## Setup
Mac / Linux:
```
uv venv --python 3.12 && uv pip install -r requirements.txt
```
Windows (no admin, no WSL; tested on the GT Makerspace PC 09/nov), GPU torch first:
```
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
uv venv --python 3.12
uv pip install torch --index-url https://download.pytorch.org/whl/cu128
uv pip install -r requirements.txt
```

## Use
```
python atlas/check.py                                  # model sanity (mass, height, collisions)
python atlas/joints_check.py                           # joints sit on their actuators
python atlas/demo.py --routine limits --video out.mp4  # 40 s range-of-motion demo, no policy
python atlas/train.py train Mjlab-Velocity-Flat-BD-Atlas --env.scene.num-envs 4096
python atlas/train.py play  Mjlab-Velocity-Flat-BD-Atlas
# carry: policy drives the 12 leg joints; waist yaw + arms are an external command, box 0.1-20 kg on the chest
python atlas/train.py Mjlab-Velocity-Carry-BD-Atlas --env.scene.num-envs 4096
# native (v2): torso heading held in the world while legs walk any way (incl. carrying "backwards"),
# waist slows with load, gait rewards fixed; fine-tunes from the carry run
python atlas/train.py Mjlab-Velocity-Native-BD-Atlas --env.scene.num-envs 4096 --agent.resume True --agent.load-run <carry run> --agent.load-checkpoint model_2999.pt
```
Fleet demo (kinematic robots, real box physics, MuJoCo 4): 2 Stretch unload two trailers, 2 Atlas carry boxes to a conveyor (MuJoCo 4 `surfacevel` belt), 2 Spot patrol. The fleet manager (`fleet/manager.py`: shared grid map, A*, task queue, traffic priority with detours, replanning on a blocked aisle or a dropped box) runs the mission; Atlas turns its torso (continuous waist) to face the pallet and the belt while its legs keep the path heading.
```
uv venv .venv-fleet --python 3.12 && uv pip install --python .venv-fleet -r requirements-fleet.txt
.venv-fleet/bin/python fleet/run.py --video fleet.mp4              # classic OpenGL renderer
.venv-fleet/bin/python fleet/run.py --video fleet.mp4 --filament   # MuJoCo 4 Filament renderer (PBR, soft shadows)
python fleet/manager.py   # manager self-check
```
mjlab training stays on MuJoCo 3.11 (`requirements.txt`); the fleet demo uses MuJoCo 4 (`requirements-fleet.txt`).

Model is generated: edit `atlas/build.py` / `atlas/cad.py`, then `python atlas/cad.py && python atlas/build.py`.
