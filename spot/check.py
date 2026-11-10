"""Spot sanity: mass vs spec, standing height, 10 s standing under joint PD (only feet may touch). Run: python spot/check.py"""

import math
from pathlib import Path

import mujoco

STAND = {"hip_y": 0.85, "knee": -1.65}
spec = mujoco.MjSpec.from_file(str(Path(__file__).parent / "spot.xml"))
spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1])
for j in spec.joints:
    if j.type != mujoco.mjtJoint.mjJNT_HINGE:
        continue
    a = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
    a.set_to_position(kp=400, kv=6)
    a.forcelimited, a.forcerange = True, ([-115, 115] if "knee" in j.name else [-45, 45])
m = spec.compile()
d = mujoco.MjData(m)
target = [STAND["hip_y"] if m.joint(m.actuator_trnid[i, 0]).name.endswith("hip_y")
          else STAND["knee"] if m.joint(m.actuator_trnid[i, 0]).name.endswith("knee") else 0.0 for i in range(m.nu)]
for i in range(m.nu):
    d.qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]] = target[i]
mujoco.mj_forward(m, d)
feet = ("front_left", "front_right", "rear_left", "rear_right")
d.qpos[2] -= min(d.site_xpos[m.site(f"{n}_foot").id][2] for n in feet)
mujoco.mj_forward(m, d)
mass = m.body_subtreemass[1]
top = d.qpos[2] + 0.065
print(f"mass {mass:.2f} kg (spec 32.7) | body top standing {top:.3f} m (spec 0.52-0.70)")
assert abs(mass - 32.7) < 0.1 and 0.52 <= top <= 0.70

worst = 0.0
while d.time < 10:
    d.ctrl[:] = target
    mujoco.mj_step(m, d)
    w, x, y, z = d.qpos[3:7]
    worst = max(worst, math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))))
touch = {m.geom(g).name for c in d.contact[: d.ncon] for g in (c.geom1, c.geom2)} - {"floor"}
print(f"10 s standing: tilt {worst:.1f} deg | body top {d.qpos[2] + 0.065:.3f} m | touching {sorted(touch)}")
assert worst < 10 and touch <= {f"{n}_foot_collision" for n in feet}
print("ok")
