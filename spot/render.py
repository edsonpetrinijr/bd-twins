"""Turntable render of spot.xml in the standing pose. Run: python spot/render.py OUT.png"""
import sys
from pathlib import Path

import mujoco
from PIL import Image

sys.path.insert(0, str(Path(__file__).parent.parent / "atlas"))
from stage import stage  # noqa: E402

spec = stage(mujoco.MjSpec.from_file(str(Path(__file__).parent / "spot.xml")))
spec.visual.global_.fovy = 25
m = spec.compile()
d = mujoco.MjData(m)
d.qpos[2], d.qpos[3] = 0.524, 1
for n in ("front_left", "front_right", "rear_left", "rear_right"):
    d.qpos[m.joint(f"{n}_hip_y").qposadr[0]] = 0.75
    d.qpos[m.joint(f"{n}_knee").qposadr[0]] = -1.5
mujoco.mj_forward(m, d)
opt = mujoco.MjvOption()
opt.geomgroup[:] = 0
opt.geomgroup[0] = opt.geomgroup[2] = 1
r = mujoco.Renderer(m, 700, 700)
tiles = []
for az, el in ((150, -15), (90, -3), (180, -5), (30, -25)):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.35]
    cam.distance, cam.azimuth, cam.elevation = 3.0, az, el
    r.update_scene(d, cam, opt)
    tiles.append(Image.fromarray(r.render()))
out = Image.new("RGB", (700 * len(tiles), 700))
for i, t in enumerate(tiles):
    out.paste(t, (700 * i, 0))
out.save(sys.argv[1])
print("saved", sys.argv[1])
