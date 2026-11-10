"""Consistency check: each joint with a visible actuator must sit on that drum's axis.
Drum axis = principal direction of its vertices (the extent that differs from the other two)."""

import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("atlas/atlas.xml")
d = mujoco.MjData(m)
d.qpos[2], d.qpos[3] = 0.999, 1
mujoco.mj_forward(m, d)

PAIRS = {"hip_roll": "hip_roll", "hip_pitch": "pelvis_drum_low", "knee": "knee_drum",
         "ankle_pitch": "ankle_drum", "shoulder_roll": "shoulder", "elbow": "elbow_drum"}


def drum_axis(g):
    mid = m.geom_dataid[g]
    a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    v = m.mesh_vert[a:a + n] @ d.geom_xmat[g].reshape(3, 3).T
    w, vec = np.linalg.eigh(np.cov((v - v.mean(0)).T))
    odd = max(range(3), key=lambda i: abs(w[i] - np.mean([w[k] for k in range(3) if k != i])))
    return vec[:, odd]


bad = 0
print(f"{'joint':26s} {'drum':16s} {'off-axis mm':>11s} {'angle deg':>9s}")
for j in range(m.njnt):
    jn = m.joint(j).name
    key = next((k for k in PAIRS if f"_{k}_joint" in jn), None)
    if key is None:
        continue
    p, a = d.xanchor[j], d.xaxis[j]
    cands = [g for g in range(m.ngeom) if m.geom_dataid[g] >= 0 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
             and m.mesh(m.geom_dataid[g]).name == PAIRS[key]]
    g = min(cands, key=lambda g: np.linalg.norm(d.geom_xpos[g] - p))
    off = np.linalg.norm(np.cross(d.geom_xpos[g] - p, a))
    ang = np.degrees(np.arccos(min(1.0, abs(np.dot(drum_axis(g), a)))))
    ok = off < 0.015 and ang < 5
    bad += not ok
    print(f"{jn:26s} {PAIRS[key]:16s} {off*1000:11.0f} {ang:9.0f}{'' if ok else '  <-- off'}")
print("all joints on their drums" if bad == 0 else f"{bad} joints off their drums")
