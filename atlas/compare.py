"""Silhouette score against the official spec-sheet render (walking Atlas, page 1).

The PDF ships an alpha mask for that render (pdfimages -png atlas-spec.pdf imgs/p -> imgs/p-004.png).
We fit camera + pose to that mask, then report IoU. Proportions are not fitted, so IoU measures shape.

Run: python atlas/compare.py IMGS_DIR OUT.png
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

IMGS, OUT = Path(sys.argv[1]), sys.argv[2]
H, W = 600, 400

ref_rgb = np.array(Image.open(IMGS / "p-003.png").convert("RGB"))
ref = np.array(Image.open(IMGS / "p-004.png")) > 128

spec = mujoco.MjSpec.from_file(str(Path(__file__).parent / "atlas.xml"))
spec.visual.global_.fovy = 25
spec.visual.global_.offwidth, spec.visual.global_.offheight = 800, 800
m = spec.compile()
d = mujoco.MjData(m)
r = mujoco.Renderer(m, H, W)
opt = mujoco.MjvOption()
opt.geomgroup[:] = 0
opt.geomgroup[2] = 1

# Pose + camera parameters fitted to the photo (start: eyeballed from p-003).
P = {"az": 118.0, "el": 2.0,
     "left_hip_pitch_joint": 0.35, "left_knee_joint": 0.45, "left_ankle_pitch_joint": -0.35,
     "right_hip_pitch_joint": -0.3, "right_knee_joint": 0.1, "right_ankle_pitch_joint": 0.2,
     "left_shoulder_pitch_joint": -0.2, "left_elbow_joint": -0.3,
     "right_shoulder_pitch_joint": 0.25, "right_elbow_joint": -0.4,
     "left_shoulder_roll_joint": 0.1, "right_shoulder_roll_joint": -0.1,
     "waist_yaw_joint": 0.0, "neck_yaw_joint": 0.0}


def render_mask(p, rgb=False):
    d.qpos[:] = 0
    d.qpos[3] = 1
    for k, v in p.items():
        if k.endswith("_joint"):
            d.qpos[m.joint(k).qposadr[0]] = v
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = d.xpos[m.body("pelvis").id] + [0, 0, -0.05]
    cam.distance, cam.azimuth, cam.elevation = 5.5, p["az"], p["el"]
    r.update_scene(d, cam, opt)
    if rgb:
        r.disable_segmentation_rendering()
        return r.render()
    r.enable_segmentation_rendering()
    seg = r.render()[..., 0]
    r.disable_segmentation_rendering()
    return seg >= 0


def bbox(mask):
    ys, xs = np.nonzero(mask)
    return ys.min(), ys.max(), xs.min(), xs.max()


def align(mask, target_shape, tb, img=None):
    """Scale/shift `mask` so its bbox height and horizontal centroid match the reference."""
    y0, y1, x0, x1 = bbox(mask)
    ty0, ty1, _, _ = tb
    s = (ty1 - ty0) / (y1 - y0)
    src = mask if img is None else img
    crop = src[y0:y1 + 1, x0:x1 + 1]
    pil = Image.fromarray(crop.astype(np.uint8) * (255 if img is None else 1))
    pil = pil.resize((max(1, int(crop.shape[1] * s)), ty1 - ty0 + 1))
    a = np.array(pil)
    cx_ref = np.nonzero(ref)[1].mean()
    msk = a > 128 if img is None else None
    cx = (np.nonzero(msk)[1].mean() if msk is not None else a.shape[1] / 2)
    out = np.zeros(target_shape + ((3,) if img is not None else ()), dtype=a.dtype)
    ox = int(round(cx_ref - cx))
    xs0, xs1 = max(0, ox), min(target_shape[1], ox + a.shape[1])
    out[ty0:ty1 + 1, xs0:xs1] = a[:, xs0 - ox:xs1 - ox]
    return (out > 128) if img is None else out, ox


def iou(p):
    a, _ = align(render_mask(p), ref.shape, bbox(ref))
    return (a & ref).sum() / (a | ref).sum()


# Coordinate descent on pose/camera only.
best = iou(P)
for step in (0.2, 0.1, 0.05, 0.025):
    improved = True
    while improved:
        improved = False
        for k in P:
            for dv in (step * (20 if k in ("az", "el") else 1), -step * (20 if k in ("az", "el") else 1)):
                q = dict(P)
                q[k] += dv
                s = iou(q)
                if s > best + 1e-4:
                    P, best, improved = q, s, True

# Region scores: split the reference bbox into head / torso+arms / legs / feet bands.
a, _ = align(render_mask(P), ref.shape, bbox(ref))
y0, y1, _, _ = bbox(ref)
hgt = y1 - y0
bands = {"head": (0.0, 0.14), "torso+arms": (0.14, 0.50), "legs": (0.50, 0.90), "feet": (0.90, 1.0)}
print(f"IoU total {best:.3f}")
for name, (f0, f1) in bands.items():
    sl = slice(y0 + int(f0 * hgt), y0 + int(f1 * hgt))
    print(f"  {name:11s} {(a[sl] & ref[sl]).sum() / max(1, (a[sl] | ref[sl]).sum()):.3f}")
print("pose", {k: round(v, 3) for k, v in P.items()})

# Overlay: white = both, red = only real Atlas (model missing), cyan = only model (model extra).
ov = np.zeros(ref.shape + (3,), np.uint8)
ov[a & ref] = (230, 230, 230)
ov[ref & ~a] = (230, 40, 40)
ov[a & ~ref] = (40, 200, 230)
rgb, _ = align(render_mask(P), ref.shape, bbox(ref), img=render_mask(P, rgb=True))
tiles = [ref_rgb, rgb, ov]
Image.fromarray(np.concatenate(tiles, axis=1)).resize((278 * 3 * 2, 861 * 2)).save(OUT)
print("saved", OUT)
