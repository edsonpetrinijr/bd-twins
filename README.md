# bd-twins

Electric Atlas (2026) in MuJoCo / mjlab, built only from public data (spec sheet, photos, videos). Every number and its source: `docs/atlas-sources.md`.

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
```
Model is generated: edit `atlas/build.py` / `atlas/cad.py`, then `python atlas/cad.py && python atlas/build.py`.
