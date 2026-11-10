"""Stretch sanity vs the BD brochure: mass, footprint, reach to the pad center, no self-collision at home.
Run: python stretch/check.py"""

from pathlib import Path

import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / "stretch.xml"))
d = mujoco.MjData(m)


def pad_center(shoulder):
    mujoco.mj_resetData(m, d)
    d.qpos[m.joint("shoulder_pitch").qposadr[0]] = shoulder
    mujoco.mj_forward(m, d)
    b = m.body("pad").id
    return d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array([0, 0, -0.29])


mass = m.body_subtreemass[m.body("base").id]
up = pad_center(np.pi)[2]
fwd = pad_center(-np.pi / 2)[0] - d.xpos[m.body("turntable").id][0]
size = 2 * m.geom_size[m.geom("base_collision").id][:2]
print(f"mass {mass:.0f} kg (1300) | footprint {size[0]:.2f} x {size[1]:.2f} m (1.25 x 1.0) | "
      f"reach up {up:.2f} m (3.2) | reach forward {fwd:.2f} m (1.95)")
assert abs(mass - 1300) < 1 and abs(up - 3.2) < 0.05 and abs(fwd - 1.95) < 0.05

spec = mujoco.MjSpec.from_file(str(Path(__file__).parent / "stretch.xml"))
spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1])
m2 = spec.compile()
d2 = mujoco.MjData(m2)
mujoco.mj_resetDataKeyframe(m2, d2, 0)
mujoco.mj_forward(m2, d2)
pairs = {tuple(sorted((m2.geom(c.geom1).name, m2.geom(c.geom2).name))) for c in d2.contact[: d2.ncon]}
self_hits = [p for p in pairs if "floor" not in p]
print("home pose contacts:", sorted(pairs))
assert not self_hits, self_hits
print("ok")
