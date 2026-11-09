"""Side-by-side sheet: official Atlas images (left) vs model from a similar camera (right), one row per view.

Run: python atlas/views.py REFS_DIR OUT.png
REFS_DIR holds the downloaded references (not committed; list in docs/atlas-sources.md).
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

REFS, OUT = Path(sys.argv[1]), sys.argv[2]
ROW_H = 520

# (reference file, crop box, camera azimuth, elevation, lookat z, distance, pose)
STAND = {"left_elbow_joint": -0.35, "right_elbow_joint": -0.35,
         "left_shoulder_roll_joint": 0.08, "right_shoulder_roll_joint": -0.08}
VIEWS = [
    ("front", "atlas-hero-2026-scaled.jpg", (1130, 90, 1830, 1241), 180, 0, 1.25, 3.2,
     {**STAND, "left_elbow_joint": -1.2, "right_elbow_joint": -1.2}),
    ("3/4 front", "Webinar-March-Atlas-form_and_funtion-Promo_Assets-0226_HP-scaled.png", (1020, 20, 1460, 790),
     145, 8, 0.95, 5.2, STAND),
    ("side", "atlas-render-blog.jpg", (800, 100, 1100, 1000), 270, 3, 0.95, 5.2, STAND),
    ("back 3/4", "atlas-battery-walk-real-collage-768x1245-1.png", (0, 0, 470, 380), 330, 10, 1.45, 2.4, STAND),
    ("head+shoulders", "pf_73.jpg", (330, 0, 1590, 1080), 180, 8, 1.62, 1.7, STAND),
]

spec = mujoco.MjSpec.from_file(str(Path(__file__).parent / "atlas.xml"))
spec.visual.global_.offwidth, spec.visual.global_.offheight = 1200, 1200
spec.visual.global_.fovy = 30
spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.1], rgba=[0.3, 0.31, 0.33, 1])
spec.worldbody.add_light(pos=[1.5, -1.5, 3.5], dir=[-0.4, 0.4, -1], diffuse=[0.9, 0.9, 0.9])
spec.worldbody.add_light(pos=[-2, 2, 3], dir=[0.5, -0.5, -1], diffuse=[0.5, 0.5, 0.55])
spec.worldbody.add_light(pos=[3, 0, 1.5], dir=[-1, 0, -0.2], diffuse=[0.4, 0.4, 0.4])
m = spec.compile()
d = mujoco.MjData(m)
opt = mujoco.MjvOption()
opt.geomgroup[:] = 0
opt.geomgroup[0] = opt.geomgroup[2] = 1

rows = []
for name, ref, box, az, el, z, dist, pose in VIEWS:
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = 0.999, 1
    for k, v in pose.items():
        d.qpos[m.joint(k).qposadr[0]] = v
    mujoco.mj_forward(m, d)
    im = Image.open(REFS / ref).convert("RGBA").convert("RGB").crop(box)
    im = im.resize((int(im.width * ROW_H / im.height), ROW_H))
    w = max(260, im.width)
    r = mujoco.Renderer(m, ROW_H, w)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, z]
    cam.distance, cam.azimuth, cam.elevation = dist, az, -el
    r.update_scene(d, cam, opt)
    model = Image.fromarray(r.render())
    r.close()
    row = Image.new("RGB", (im.width + w + 10, ROW_H))
    row.paste(im, (0, 0))
    row.paste(model, (im.width + 10, 0))
    rows.append(row)

W = max(r.width for r in rows)
sheet = Image.new("RGB", (W, sum(r.height + 10 for r in rows)))
y = 0
for row in rows:
    sheet.paste(row, (0, y))
    y += row.height + 10
sheet.save(OUT)
print("saved", OUT)
