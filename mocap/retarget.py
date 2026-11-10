"""Retarget a CMU mocap BVH clip to Atlas -> CSV for mjlab motion tracking (BeyondMimic).

Data: CMU Graphics Lab Motion Capture Database, BVH conversion by B. Hahne (mocap/raw/, see
docs/mocap-sources.md). "Free for use in research and commercial projects."

Method (no extra deps): BVH forward kinematics -> human joint positions, mapped to MuJoCo axes and scaled
by leg length -> per-frame damped-least-squares IK on atlas.xml (pelvis pose, torso yaw, ankles + foot yaw,
knees, wrists, elbows, head), warm-started from the previous frame and clamped to joint limits.

Output CSV row (mjlab / Unitree convention): base pos (3), base quat xyzw (4), joint angles in model order.
  python mocap/retarget.py mocap/raw/69_70.bvh --start 1 --end 1045 --out mocap/atlas_69_70.csv [--video v.mp4]
"""

import argparse
import math
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
ATLAS = ROOT / "atlas" / "atlas.xml"
FPS_OUT = 30


# ---------------------------------------------------------------- BVH
def parse_bvh(path):
    toks = Path(path).read_text().split()
    i, joints, stack = 0, [], []
    while toks[i] != "MOTION":
        t = toks[i]
        if t in ("ROOT", "JOINT"):
            joints.append({"name": toks[i + 1], "parent": stack[-1] if stack else -1, "chan": []})
            i += 2
        elif t == "End":
            joints.append({"name": joints[stack[-1]]["name"] + "_end", "parent": stack[-1], "chan": []})
            i += 2
        elif t == "{":
            stack.append(len(joints) - 1)
            i += 1
        elif t == "}":
            stack.pop()
            i += 1
        elif t == "OFFSET":
            joints[stack[-1]]["offset"] = np.array([float(x) for x in toks[i + 1:i + 4]])
            i += 4
        elif t == "CHANNELS":
            n = int(toks[i + 1])
            joints[stack[-1]]["chan"] = toks[i + 2:i + 2 + n]
            i += 2 + n
        else:
            i += 1
    nframes = int(toks[i + 2])
    dt = float(toks[i + 5])
    data = np.array(toks[i + 6:], dtype=float).reshape(nframes, -1)
    return joints, data, dt


def rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return {"X": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "Y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "Z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def bvh_fk(joints, frame):
    """Global positions and rotations of every joint for one frame (BVH axes, BVH units)."""
    pos, R, k = [], [], 0
    for j in joints:
        p, r = np.zeros(3), np.eye(3)
        for c in j["chan"]:
            v = frame[k]
            k += 1
            if c.endswith("position"):
                p["XYZ".index(c[0])] = v
            else:
                r = r @ rot(c[0], v)
        if j["parent"] < 0:
            pos.append(p + j.get("offset", 0))
            R.append(r)
        else:
            P, RP = pos[j["parent"]], R[j["parent"]]
            pos.append(P + RP @ j["offset"])
            R.append(RP @ r)
    return np.array(pos), np.array(R)


# BVH (y up, character faces +z, left = +x) -> MuJoCo (z up, faces +x, left = +y)
B2M = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)


# ---------------------------------------------------------------- IK
class AtlasIK:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(ATLAS))
        self.d = mujoco.MjData(self.m)
        self.m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        self.lo = np.full(self.m.nq, -np.inf)
        self.hi = np.full(self.m.nq, np.inf)
        for j in range(self.m.njnt):
            if self.m.jnt_limited[j]:
                a = self.m.jnt_qposadr[j]
                self.lo[a], self.hi[a] = self.m.jnt_range[j]
        self.d.qpos[2] = 0.975
        self.d.qpos[3] = 1
        mujoco.mj_forward(self.m, self.d)

    def body(self, n):
        return self.m.body(n).id

    def solve(self, targets, iters=40, damping=1e-2):
        """targets: list of (body, local point, world pos or None, world rot or None, weight)."""
        m, d = self.m, self.d
        for _ in range(iters):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            J, e = [], []
            for b, lp, wp, wr, w in targets:
                bid = self.body(b)
                xr = d.xmat[bid].reshape(3, 3)
                if wp is not None:
                    pt = d.xpos[bid] + xr @ lp
                    jp = np.zeros((3, m.nv))
                    mujoco.mj_jac(m, d, jp, None, pt, bid)
                    J.append(w * jp)
                    e.append(w * (wp - pt))
                if wr is not None:
                    jr = np.zeros((3, m.nv))
                    mujoco.mj_jacBody(m, d, None, jr, bid)
                    q, qt, er = np.zeros(4), np.zeros(4), np.zeros(3)
                    mujoco.mju_mat2Quat(q, xr.flatten())
                    mujoco.mju_mat2Quat(qt, wr.flatten())
                    mujoco.mju_subQuat(er, qt, q)  # body-frame error
                    J.append(w * jr)
                    e.append(w * (xr @ er))
            J, e = np.vstack(J), np.concatenate(e)
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(len(e)), e)
            mujoco.mj_integratePos(m, d.qpos, dq, 1.0)
            d.qpos[:] = np.clip(d.qpos, self.lo, self.hi)
        mujoco.mj_kinematics(m, d)
        return d.qpos.copy()


def yaw_mat(v):
    yaw = math.atan2(v[1], v[0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def write_npz(m, qs, path, fps_out=50):
    """Same arrays as mjlab's csv_to_npz: joint pos/vel in model order, every body (model order, no world)
    link pose and world velocity at the body origin. Base and joints are interpolated from 30 to 50 fps."""
    t_in = np.arange(len(qs)) / FPS_OUT
    t_out = np.arange(0, t_in[-1], 1 / fps_out)
    q = np.stack([np.interp(t_out, t_in, qs[:, i]) for i in range(qs.shape[1])], axis=1)
    q[:, 3:7] /= np.linalg.norm(q[:, 3:7], axis=1, keepdims=True)
    d = mujoco.MjData(m)
    out = {k: [] for k in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w",
                           "body_ang_vel_w")}
    for k in range(len(q)):
        a, b = q[max(k - 1, 0)], q[min(k + 1, len(q) - 1)]
        v = np.zeros(m.nv)
        mujoco.mj_differentiatePos(m, v, (min(k + 1, len(q) - 1) - max(k - 1, 0)) / fps_out, a, b)
        d.qpos[:], d.qvel[:] = q[k], v
        mujoco.mj_forward(m, d)
        vel = np.zeros((m.nbody - 1, 6))
        for bid in range(1, m.nbody):
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, bid, vel[bid - 1], 0)
        out["joint_pos"].append(q[k, 7:].copy())
        out["joint_vel"].append(v[6:].copy())
        out["body_pos_w"].append(d.xpos[1:].copy())
        out["body_quat_w"].append(d.xquat[1:].copy())
        out["body_ang_vel_w"].append(vel[:, :3])
        out["body_lin_vel_w"].append(vel[:, 3:])
    np.savez(path, fps=[fps_out], **{k: np.stack(v) for k, v in out.items()})
    print(f"wrote {path}: {len(q)} frames @ {fps_out} fps, {m.nbody - 1} bodies")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bvh")
    ap.add_argument("--start", type=int, default=1, help="first frame (frame 0 is the added T pose)")
    ap.add_argument("--end", type=int)
    ap.add_argument("--out", required=True)
    ap.add_argument("--video")
    ap.add_argument("--npz", help="also write the mjlab tracking motion file (50 fps, no W&B needed)")
    args = ap.parse_args()

    joints, data, dt = parse_bvh(args.bvh)
    idx = {j["name"]: k for k, j in enumerate(joints)}
    step = max(1, round(1 / (dt * FPS_OUT)))  # 120 fps -> 30 fps
    frames = data[args.start:args.end:step]

    ik = AtlasIK()
    # scale: human hip->ankle to Atlas hip->ankle (thigh + shin)
    p0, _ = bvh_fk(joints, data[0])
    leg_h = np.linalg.norm(p0[idx["LeftLeg"]] - p0[idx["LeftUpLeg"]]) + np.linalg.norm(
        p0[idx["LeftFoot"]] - p0[idx["LeftLeg"]])
    s = (0.467 + 0.336) / leg_h

    H = [bvh_fk(joints, f) for f in frames]
    P = np.array([(B2M @ p.T).T * s for p, _ in H])
    Rg = np.array([[B2M @ r @ B2M.T for r in R] for _, R in H])
    # ground: the lower ankle should sit at Atlas' ankle height (0.196 m) when a foot is planted
    ank = np.minimum(P[:, idx["LeftFoot"], 2], P[:, idx["RightFoot"], 2])
    P[:, :, 2] += 0.196 - np.percentile(ank, 5)
    # Atlas hips are wider than the human pelvis offset; IK handles it (feet weights > hip placement)

    # T-pose rest rotations, so "rotation relative to rest" is what we transfer to Atlas bodies
    _, R0 = bvh_fk(joints, data[0])
    R0 = np.array([B2M @ r @ B2M.T for r in R0])

    rows, qs = [], []
    for k in range(len(frames)):
        p, R = P[k], Rg[k]
        hip_R = R[idx["Hips"]] @ R0[idx["Hips"]].T
        torso_R = R[idx["Spine1"]] @ R0[idx["Spine1"]].T
        T = []
        T.append(("pelvis", np.zeros(3), p[idx["Hips"]] + hip_R @ np.array([0, 0, 0.0]), hip_R, 3.0))
        T.append(("torso", np.zeros(3), None, torso_R, 1.0))
        for side, S in (("left", "Left"), ("right", "Right")):
            foot_dir = p[idx[f"{S}ToeBase"]] - p[idx[f"{S}Foot"]]
            T.append((f"{side}_foot", np.zeros(3), p[idx[f"{S}Foot"]], yaw_mat(foot_dir), 5.0))
            T.append((f"{side}_shin", np.zeros(3), p[idx[f"{S}Leg"]], None, 1.0))
            T.append((f"{side}_hand", np.zeros(3), p[idx[f"{S}Hand"]], None, 1.0))
            T.append((f"{side}_forearm", np.zeros(3), p[idx[f"{S}ForeArm"]], None, 0.5))
        T.append(("head", np.zeros(3), p[idx["Head"]], None, 0.3))
        q = ik.solve(T, iters=60 if k == 0 else 20)
        qs.append(q)
        w, x, y, z = q[3:7]
        rows.append(np.concatenate([q[:3], [x, y, z, w], q[7:]]))

    out = Path(args.out)
    np.savetxt(out, np.array(rows), delimiter=",", fmt="%.6f")
    print(f"wrote {out}: {len(rows)} frames @ {FPS_OUT} fps, scale {s:.4f}, nq {ik.m.nq}")

    # sanity: foot penetration / float, pelvis height range
    m, d = ik.m, ik.d
    soles = []
    for q in qs:
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        soles.append(min(d.site("left_foot").xpos[2], d.site("right_foot").xpos[2]))
    print(f"lowest sole z: min {min(soles):.3f} median {np.median(soles):.3f}; "
          f"pelvis z {min(q[2] for q in qs):.2f}-{max(q[2] for q in qs):.2f}")

    if args.npz:
        write_npz(ik.m, np.array(qs), args.npz)

    if args.video:
        import subprocess
        spec = mujoco.MjSpec.from_file(str(ATLAS))
        spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[10, 10, 0.1], rgba=[0.5, 0.5, 0.52, 1])
        spec.worldbody.add_light(pos=[0, 0, 6], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
        mv = spec.compile()
        dv = mujoco.MjData(mv)
        r = mujoco.Renderer(mv, 480, 640)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 4.0, -12, 120
        ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
                               "640x480", "-r", str(FPS_OUT), "-i", "-", "-pix_fmt", "yuv420p", args.video],
                              stdin=subprocess.PIPE)
        for q in qs:
            dv.qpos[:] = q
            mujoco.mj_forward(mv, dv)
            cam.lookat[:] = [q[0], q[1], 0.9]
            r.update_scene(dv, cam)
            ff.stdin.write(r.render().tobytes())
        ff.stdin.close()
        ff.wait()
        print("video", args.video)


if __name__ == "__main__":
    main()
