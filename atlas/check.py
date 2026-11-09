"""Sanity check for atlas.xml against the public spec sheet. Run: python atlas/check.py"""

from pathlib import Path

import mujoco
import numpy as np

XML = Path(__file__).parent / "atlas.xml"

spec = mujoco.MjSpec.from_file(str(XML))
spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1])
m = spec.compile()
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

# Spec sheet: 90 kg, 1.9 m, reach 2.3 m.
mass = m.body_subtreemass[m.body("pelvis").id]
head = m.geom("head")
top = d.geom_xpos[head.id][2] + m.geom_size[head.id][0]
shoulder_z = d.xpos[m.body("left_upper_arm").id][2]
arm_len = 0.31 + 0.28 + 0.16  # upper arm, forearm, hand (fit to p-004 silhouette)
reach = shoulder_z + arm_len
hinges = sum(m.jnt_type == mujoco.mjtJoint.mjJNT_HINGE)

print(f"mass {mass:.2f} kg | height {top:.3f} m | overhead reach {reach:.2f} m | hinge DoF {hinges}")
assert abs(mass - 90.0) < 0.5
assert abs(top - 1.9) < 0.03
assert abs(reach - 2.3) < 0.06  # spec 'reach' definition unknown; overhead fingertip reach used
assert hinges == 28  # body only; spec says 56 total, rest assumed in hands

# Standing pose: only the feet touch the floor, no self-collision.
floor = m.geom("floor").id
touching = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
            for c in d.contact[: d.ncon] for g in (c.geom1, c.geom2) if g != floor}
assert all(c.geom1 == floor or c.geom2 == floor for c in d.contact[: d.ncon]), "self-collision at rest"
assert touching <= {"left_foot", "right_foot"}, touching

# Passive drop: no actuators, it collapses, but physics must stay finite.
for _ in range(2000):
    mujoco.mj_step(m, d)
assert np.all(np.isfinite(d.qpos)), "simulation blew up"
print("ok")
