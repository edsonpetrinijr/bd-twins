"""Renders stretch.xml at the home pose from 4 angles. Run: python stretch/render.py OUT.png"""
import sys
from pathlib import Path

import mujoco
from PIL import Image

sys.path.insert(0, str(Path(__file__).parent.parent / "atlas"))
from stage import stage  # noqa: E402

spec = stage(mujoco.MjSpec.from_file(str(Path(__file__).parent / "stretch.xml")))
spec.visual.global_.fovy = 30
m = spec.compile()
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
mujoco.mj_forward(m, d)
opt = mujoco.MjvOption()
opt.geomgroup[:] = 0
opt.geomgroup[0] = opt.geomgroup[2] = 1
r = mujoco.Renderer(m, 800, 640)
tiles = []
for az, el in ((150, -12), (90, -5), (200, -10), (30, -20)):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.2, 0, 1.0]
    cam.distance, cam.azimuth, cam.elevation = 5.5, az, el
    r.update_scene(d, cam, opt)
    tiles.append(Image.fromarray(r.render()))
out = Image.new("RGB", (640 * len(tiles), 800))
for i, t in enumerate(tiles):
    out.paste(t, (640 * i, 0))
out.save(sys.argv[1])
print("saved", sys.argv[1])
