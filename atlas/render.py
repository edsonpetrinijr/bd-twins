"""Renders atlas.xml (visual geoms only) next to the spec-sheet photos. Run: python atlas/render.py OUT.png"""

import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

spec = mujoco.MjSpec.from_file(str(Path(__file__).parent / "atlas.xml"))
spec.visual.global_.offwidth, spec.visual.global_.offheight = 1200, 1200
spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.1], rgba=[0.35, 0.36, 0.38, 1])
spec.worldbody.add_light(pos=[1.5, -1.5, 3.5], dir=[-0.4, 0.4, -1], diffuse=[0.8, 0.8, 0.8])
spec.worldbody.add_light(pos=[-2, 2, 3], dir=[0.5, -0.5, -1], diffuse=[0.4, 0.4, 0.45])
m = spec.compile()
d = mujoco.MjData(m)

# Pose roughly like the walking render on page 1 of the spec sheet.
pose = {"left_hip_pitch_joint": -0.45, "left_knee_joint": 0.25, "left_ankle_pitch_joint": 0.15,
        "right_hip_pitch_joint": 0.3, "right_knee_joint": 0.35, "right_ankle_pitch_joint": -0.2,
        "left_shoulder_pitch_joint": 0.35, "right_shoulder_pitch_joint": -0.3,
        "left_elbow_joint": -0.5, "right_elbow_joint": -0.35}
for name, q in pose.items():
    d.qpos[m.joint(name).qposadr[0]] = q
mujoco.mj_forward(m, d)

opt = mujoco.MjvOption()
opt.geomgroup[:] = 0
opt.geomgroup[0] = opt.geomgroup[2] = 1
r = mujoco.Renderer(m, 900, 520)
views = []
for az, el in ((160, -5), (90, 0), (180, -15)):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.95]
    cam.distance, cam.azimuth, cam.elevation = 2.7, az, el
    r.update_scene(d, cam, opt)
    views.append(Image.fromarray(r.render()))

refs = Path(sys.argv[2]) if len(sys.argv) > 2 else None
tiles = []
if refs:
    for name in ("p-003.png", "p-008.png"):
        im = Image.open(refs / name).convert("RGB")
        tiles.append(im.resize((int(im.width * 900 / im.height), 900)))
tiles += views
out = Image.new("RGB", (sum(t.width for t in tiles), 900))
x = 0
for t in tiles:
    out.paste(t, (x, 0))
    x += t.width
out.save(sys.argv[1])
print("saved", sys.argv[1])
