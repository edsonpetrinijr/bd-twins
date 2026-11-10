"""Scripted demo, no policy: Atlas stands, squats, waves, turns at the waist. Joint PD only.

  python atlas/demo.py                 # headless: prints whether it stayed up
  python atlas/demo.py --video out.mp4 # also writes a video (ffmpeg)
  mjpython atlas/demo.py --viewer      # interactive MuJoCo viewer (Mac needs mjpython)

Torque limits are the estimates from atlas_mjlab.py. Gains here are stiffer than the RL ones on purpose:
with no policy, only joint stiffness keeps it up.
"""

import argparse
import math
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from stage import stage  # noqa: E402

# (joint regex-ish prefix, kp Nm/rad, kd Nms/rad, torque limit Nm)
GAINS = [
    (("hip_pitch", "knee"), 1200, 40, 400), (("hip_roll",), 1200, 40, 300), (("hip_yaw",), 400, 20, 150),
    (("ankle_pitch",), 2500, 60, 250), (("ankle_roll",), 1500, 40, 120), (("waist",), 600, 30, 200),
    (("shoulder_pitch", "shoulder_roll"), 200, 10, 200), (("shoulder_yaw",), 100, 5, 100),
    (("elbow",), 100, 5, 120), (("wrist",), 20, 1, 30), (("neck",), 20, 1, 10),
]
STAND = {"hip_pitch": -0.25, "knee": 0.5, "ankle_pitch": -0.25, "elbow": -0.35}


def build():
    spec = stage(mujoco.MjSpec.from_file(str(Path(__file__).parent / "atlas.xml")))
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for j in spec.joints:
        if j.type != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        kp, kd, lim = next((kp, kd, lim) for keys, kp, kd, lim in GAINS if any(k in j.name for k in keys))
        a = spec.add_actuator(name=j.name.replace("_joint", ""), target=j.name,
                              trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_position(kp=kp, kv=kd)
        a.forcelimited, a.forcerange = True, [-lim, lim]
    return spec.compile()


def targets(m, t):
    """Joint targets at time t (s)."""
    q = {}
    squat = 0.0 if t < 2 else 0.2 * (1 - math.cos(2 * math.pi * (t - 2) / 4)) if t < 6 else 0.0
    knee = STAND["knee"] + squat * 2
    for side, s in (("left", 1), ("right", -1)):
        q[f"{side}_knee_joint"] = knee
        q[f"{side}_hip_pitch_joint"] = -knee / 2 - 0.6 * squat  # lean the torso forward to keep the COM over the feet
        q[f"{side}_ankle_pitch_joint"] = -knee / 2
        q[f"{side}_elbow_joint"] = STAND["elbow"]
        q[f"{side}_shoulder_roll_joint"] = s * 0.08
    if 6 <= t < 9:  # wave right hand
        q["right_shoulder_roll_joint"] = -1.4
        q["right_elbow_joint"] = -1.2 + 0.5 * math.sin(2 * math.pi * 1.5 * (t - 6))
    if 9 <= t < 12:  # look around: waist and head
        q["waist_yaw_joint"] = 0.6 * math.sin(2 * math.pi * (t - 9) / 3)
        q["neck_yaw_joint"] = -0.8 * math.sin(2 * math.pi * (t - 9) / 3)
    return q


def ease(x):
    x = min(1.0, max(0.0, x))
    return 0.5 - 0.5 * math.cos(math.pi * x)


def targets_limits(m, t):
    """Range-of-motion tour: waist 360, head 360, arm windmill, arms overhead, deep squat,
    bow, then the body spins back while the head keeps facing forward. (Side sway removed: the
    ankle-only balance cannot hold lateral shifts; a policy should.)"""
    q = targets(m, 0)
    waist = 2 * math.pi * ease((t - 2) / 6)                 # 2-8 s: torso turns all the way around
    neck = -2 * math.pi * ease((t - 8) / 5)                  # 8-13 s: head turns all the way around
    if t >= 32:                                              # 32-38 s: torso spins back, head holds still
        waist = 2 * math.pi * (1 - ease((t - 32) / 6))
        neck = -waist
    q["waist_yaw_joint"], q["neck_yaw_joint"] = waist, neck
    if 13 <= t < 19:                                         # arm windmill, opposite phase, elbows straight
        w = ease((t - 13) / 0.8) * (1 - ease((t - 18.2) / 0.8))
        ph = 2 * math.pi * (t - 13) / 2.5
        q["left_shoulder_pitch_joint"] = -3.0 * w * math.sin(ph)
        q["right_shoulder_pitch_joint"] = 3.0 * w * math.sin(ph)
        q["left_elbow_joint"] = q["right_elbow_joint"] = -0.05
        q["left_wrist_yaw_joint"] = q["right_wrist_yaw_joint"] = 2.5 * w * math.sin(2 * ph)
    if 19 <= t < 23:                                         # arms up over the head (shoulder roll limit)
        up = ease((t - 19) / 1.5) * (1 - ease((t - 21.5) / 1.5))
        q["left_shoulder_roll_joint"] = 0.08 + 2.85 * up
        q["right_shoulder_roll_joint"] = -0.08 - 2.85 * up
        q["left_elbow_joint"] = q["right_elbow_joint"] = -0.35 * (1 - up)
    if 23 <= t < 28:                                         # deep squat
        sq = ease((t - 23) / 2.2) * (1 - ease((t - 25.6) / 2.2))
        knee = STAND["knee"] + 1.0 * sq
        for side in ("left", "right"):
            q[f"{side}_knee_joint"] = knee
            q[f"{side}_hip_pitch_joint"] = -knee / 2 - 0.55 * sq
            q[f"{side}_ankle_pitch_joint"] = -knee / 2
        q["left_shoulder_pitch_joint"] = q["right_shoulder_pitch_joint"] = -1.3 * sq  # arms forward for balance
    if 28 <= t < 32:                                         # bow: fold at the hips, arms reach down
        bow = ease((t - 28) / 1.6) * (1 - ease((t - 30.4) / 1.6))
        for side in ("left", "right"):
            q[f"{side}_hip_pitch_joint"] -= 0.7 * bow
        q["left_shoulder_pitch_joint"] = q["right_shoulder_pitch_joint"] = -0.9 * bow
        q["neck_yaw_joint"] = q["neck_yaw_joint"]  # head keeps its last angle
    return q


ROUTINES = {"basic": (targets, 12.0), "limits": (targets_limits, 40.0)}
ROUTINE = targets


def reset(m, d):
    mujoco.mj_resetData(m, d)
    d.qpos[2] = 0.975
    for name, v in targets(m, 0).items():
        d.qpos[m.joint(name).qposadr[0]] = v
    mujoco.mj_forward(m, d)


def control(m, d):
    """Scripted joint targets + a simple balance term: ankles lean the body so the COM stays over the feet."""
    q = ROUTINE(m, d.time)
    feet = (d.site_xpos[m.site("left_foot").id] + d.site_xpos[m.site("right_foot").id]) / 2
    err = d.subtree_com[1, :2] - feet[:2]
    vel = d.subtree_linvel[1, :2]
    pitch = 1.5 * err[0] + 0.3 * vel[0]   # COM behind the feet -> lean forward
    roll = -1.5 * err[1] - 0.3 * vel[1]
    for side in ("left", "right"):
        q[f"{side}_ankle_pitch_joint"] = q.get(f"{side}_ankle_pitch_joint", 0.0) + pitch
        q[f"{side}_ankle_roll_joint"] = q.get(f"{side}_ankle_roll_joint", 0.0) + roll
    for i in range(m.nu):
        jn = m.joint(m.actuator_trnid[i, 0]).name
        d.ctrl[i] = q.get(jn, 0.0)


def tilt_deg(d):
    w, x, y, z = d.qpos[3:7]
    up_z = 1 - 2 * (x * x + y * y)  # z of the pelvis up axis
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--routine", choices=ROUTINES, default="basic")
    ap.add_argument("--seconds", type=float)
    args = ap.parse_args()
    global ROUTINE
    ROUTINE, default_s = ROUTINES[args.routine]
    args.seconds = args.seconds or default_s
    m = build()
    d = mujoco.MjData(m)
    reset(m, d)

    if args.viewer:
        from mujoco import viewer as mjviewer
        mujoco.set_mjcb_control(control)
        mjviewer.launch(m, d)
        return

    fps, W, H = 30, 1280, 720
    if args.video:
        r = mujoco.Renderer(m, H, W)
        opt = mujoco.MjvOption()
        opt.geomgroup[:] = 0
        opt.geomgroup[0] = opt.geomgroup[2] = 1
        cam = mujoco.MjvCamera()
        cam.distance, cam.azimuth, cam.elevation = 3.0, 145, -8
        ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                               "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", args.video],
                              stdin=subprocess.PIPE)
    worst, low = 0.0, 9.0
    next_frame = 0.0
    while d.time < args.seconds:
        control(m, d)
        mujoco.mj_step(m, d)
        worst, low = max(worst, tilt_deg(d)), min(low, d.qpos[2])
        if args.video and d.time >= next_frame:
            cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.95]
            if args.routine == "limits":
                cam.azimuth = 145 + 4 * d.time  # slow orbit
            r.update_scene(d, cam, opt)
            ff.stdin.write(r.render().tobytes())
            next_frame += 1 / fps
    if args.video:
        ff.stdin.close()
        ff.wait()
    fell = low < 0.5 or worst > 45
    print(f"{args.seconds:.0f} s | max torso tilt {worst:.1f} deg | lowest pelvis {low:.2f} m | "
          f"{'FELL' if fell else 'stayed up'}")
    assert np.all(np.isfinite(d.qpos)), "simulation blew up"


if __name__ == "__main__":
    main()
