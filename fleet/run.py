"""Fleet demo, phase 1 ("fake" robots): the three models are posed kinematically (no leg physics),
driven by fleet/manager.py. Stretch unloads a trailer onto a staging pallet, Atlas carries boxes to the
line, Spot patrols. Mid-run an aisle gets blocked (Spot sees it, everyone replans) and Atlas drops a box
(recover task).

  python fleet/run.py                  # headless, prints the event log, checks every box got delivered
  python fleet/run.py --video out.mp4  # plus a video with a status panel
"""

import argparse
import math
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "fleet"))
sys.path.insert(0, str(ROOT / "atlas"))
from manager import FleetManager, GridMap, Robot  # noqa: E402
from stage import stage  # noqa: E402

N_BOXES = 4
TRUCK = (-6.0, 0.0)                       # trailer center
STRETCH_POS, STRETCH_YAW = (-3.6, 0.0), math.pi
STAGING = (-3.1, 1.75)                    # pallet center
PICKUP = (-2.0, 1.75)                     # where Atlas stands to take a box from the staging pallet
LINE, DROPOFF = (6.6, 0.0), (5.4, 0.0)
OBSTACLE = (2.0, 0.0)
PATROL = [(5.0, -3.2), (5.0, 3.2), (0.5, 3.2), (-0.8, 0.0), (0.5, -3.2)]
BOX = (0.35, 0.3, 0.25)                   # full size, m


def quat_yaw(yaw):
    return [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


def build_scene():
    w = stage(mujoco.MjSpec(), floor=(0.42, 0.43, 0.45, 1))
    w.visual.global_.fovy = 40
    add = w.worldbody.add_geom
    add(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[10, 7, 0.1], pos=(0, 0, -0.001), rgba=[0.42, 0.43, 0.45, 1])  # warehouse floor
    gray, dark, card = [0.55, 0.57, 0.6, 1], [0.2, 0.21, 0.23, 1], [0.72, 0.55, 0.36, 1]
    # trailer: floor, two side walls, roof, open toward +x
    # trailer: floor, side walls and front wall (no roof, walls see-through so the camera can look in)
    for pos, size in (((-6.0, 0, 0.02), (1.6, 1.25, 0.02)), ((-6.0, 1.25, 1.3), (1.6, 0.03, 1.3)),
                      ((-6.0, -1.25, 1.3), (1.6, 0.03, 1.3)), ((-7.6, 0, 1.3), (0.03, 1.25, 1.3))):
        add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size, rgba=[0.78, 0.8, 0.83, 0.35])
    for x in (-5.0, 0.0, 5.0):  # warehouse ceiling lights
        w.worldbody.add_light(pos=[x, 0, 7], dir=[0, 0, -1], diffuse=[0.35, 0.35, 0.33], castshadow=False)
    add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(*STAGING, 0.07), size=(0.55, 0.45, 0.07), rgba=[0.6, 0.45, 0.25, 1])
    add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(*LINE, 0.45), size=(0.45, 1.2, 0.45), rgba=dark)        # conveyor
    add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(*LINE, 0.92), size=(0.42, 1.18, 0.02), rgba=[0.1, 0.1, 0.1, 1])
    for y in (-1.6, 1.6):                                                                          # shelving
        for z in (0.03, 0.55, 1.07):  # open shelving: three decks on corner posts
            add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(2.0, y, z), size=(2.5, 0.4, 0.03), rgba=[0.2, 0.35, 0.65, 1])
        for px in (-0.45, 4.45):
            for py in (y - 0.37, y + 0.37):
                add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(px, py, 0.55), size=(0.03, 0.03, 0.55), rgba=gray)
        for k in range(6):
            add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(0.0 + 0.8 * k, y, 0.26 + 0.52 * (k % 2)),
                size=(0.25, 0.3, 0.2), rgba=card)
    for i, (x, y) in enumerate(((-2.0, -4.0), (6.0, -4.0))):                                        # floor lines
        add(type=mujoco.mjtGeom.mjGEOM_BOX, pos=(2.0, -4.5 + 9 * i, 0.002), size=(6.0, 0.05, 0.002),
            rgba=[0.95, 0.75, 0.1, 1])
    obstacle = w.worldbody.add_body(name="obstacle", pos=(0, 0, -5))                               # appears later
    obstacle.add_freejoint(name="obstacle")
    obstacle.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=(0.5, 0.45, 0.4), pos=(0, 0, 0.4), rgba=card)
    for i in range(N_BOXES):
        b = w.worldbody.add_body(name=f"box{i}", pos=(0, 0, 0))
        b.add_freejoint(name=f"box{i}")
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[s / 2 for s in BOX], rgba=card)
    for name, xml, meshdir in (("atlas", "atlas/atlas.xml", "atlas"), ("spot", "spot/spot.xml", "spot/meshes"),
                               ("stretch", "stretch/stretch.xml", "stretch/meshes")):
        child = mujoco.MjSpec.from_file(str(ROOT / xml))
        child.meshdir = str(ROOT / meshdir)
        w.worldbody.add_frame().attach_body(child.worldbody.first_body(), f"{name}/", "")
    return w.compile()


class Poser:
    """Writes robot and box poses into qpos (kinematic playback)."""

    def __init__(self, m, d):
        self.m, self.d = m, d

    def q(self, joint, value):
        self.d.qpos[self.m.joint(joint).qposadr[0]] = value

    def free(self, joint, pos, quat=(1, 0, 0, 0)):
        a = self.m.joint(joint).qposadr[0]
        self.d.qpos[a:a + 3], self.d.qpos[a + 3:a + 7] = pos, quat

    def atlas(self, x, y, yaw, phase, moving, carrying):
        self.free("atlas/floating_base_joint", (x, y, 0.975 + (0.015 * abs(math.sin(phase)) if moving else 0)),
                  quat_yaw(yaw))
        for side, s in (("left", 1), ("right", -1)):
            swing = 0.35 * math.sin(phase + (0 if s > 0 else math.pi)) if moving else 0.0
            lift = 0.45 * max(0.0, math.sin(phase + (0 if s > 0 else math.pi))) if moving else 0.0
            knee = 0.5 + lift
            self.q(f"atlas/{side}_hip_pitch_joint", -knee / 2 - swing)
            self.q(f"atlas/{side}_knee_joint", knee)
            self.q(f"atlas/{side}_ankle_pitch_joint", -knee / 2 + swing * 0.5)
            if carrying:
                self.q(f"atlas/{side}_shoulder_pitch_joint", -1.05)
                self.q(f"atlas/{side}_shoulder_roll_joint", s * 0.12)
                self.q(f"atlas/{side}_elbow_joint", -1.1)
            else:
                self.q(f"atlas/{side}_shoulder_pitch_joint", -swing * 0.8)
                self.q(f"atlas/{side}_shoulder_roll_joint", s * 0.08)
                self.q(f"atlas/{side}_elbow_joint", -0.35)

    def spot(self, x, y, yaw, phase, moving):
        self.free("spot/floating_base_joint", (x, y, 0.49), quat_yaw(yaw))
        for leg, ph in (("front_left", 0), ("rear_right", 0), ("front_right", math.pi), ("rear_left", math.pi)):
            s = math.sin(phase + ph) if moving else 0.0
            self.q(f"spot/{leg}_hip_y", 0.85 + 0.25 * s)
            self.q(f"spot/{leg}_knee", -1.65 - 0.35 * max(0.0, s))

    def stretch_arm(self, target):
        """Turntable yaw + 2-link IK so the vertical pad faces `target` (world xyz of the box face)."""
        m, d = self.m, self.d
        tt = d.xpos[m.body("stretch/turntable").id]
        base_yaw = d.qpos[m.joint("stretch/base_yaw").qposadr[0]]
        dx, dy = target[0] - tt[0], target[1] - tt[1]
        yaw = math.atan2(dy, dx) - base_yaw
        R = math.hypot(dx, dy) - 0.05 - 0.08                        # shoulder sits 5 cm ahead; pad face 8 cm out
        Z = target[2] + 0.29 + 0.16 - (tt[2] + 0.70)               # wrist center height vs shoulder
        L1, L2 = 0.75, 0.70
        D = min(math.hypot(R, Z), L1 + L2 - 1e-3)
        a = math.atan2(Z, R)
        b = math.acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D))
        a1 = a + b                                                   # elbow up
        e = (D * math.cos(a) - L1 * math.cos(a1), D * math.sin(a) - L1 * math.sin(a1))
        th1 = math.atan2(-L1 * math.cos(a1), -L1 * math.sin(a1))
        th12 = math.atan2(-e[0], -e[1])
        self.q("stretch/turntable_yaw", (yaw + math.pi) % (2 * math.pi) - math.pi)
        self.q("stretch/shoulder_pitch", th1)
        self.q("stretch/elbow_pitch", th12 - th1)
        self.q("stretch/wrist_pitch", -th12)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video")
    ap.add_argument("--seconds", type=float, default=110)
    args = ap.parse_args()

    m = build_scene()
    d = mujoco.MjData(m)
    P = Poser(m, d)

    grid = GridMap(-8, -5, 8, 5)
    grid.block_rect(-7.6, -1.3, -4.4, 1.3)                       # trailer
    grid.block_rect(-4.3, -0.6, -2.9, 0.6)                       # Stretch
    grid.block_rect(STAGING[0] - 0.55, STAGING[1] - 0.45, STAGING[0] + 0.55, STAGING[1] + 0.45)
    grid.block_rect(LINE[0] - 0.45, -1.2, LINE[0] + 0.45, 1.2)   # conveyor
    for y in (-1.6, 1.6):
        grid.block_rect(-0.5, y - 0.4, 4.5, y + 0.4, inflate=0.25)
    atlas = Robot("Atlas", "atlas", [-1.0, 3.3], 1.0, {"carry", "recover"})
    spot = Robot("Spot", "spot", [5.0, -3.2], 1.4, {"inspect"})
    stretch = Robot("Stretch", "stretch", list(STRETCH_POS), 0.0, {"unload"})
    fm = FleetManager(grid, [atlas, spot, stretch], staging=PICKUP, line=DROPOFF, patrol=PATROL)

    truck_slots = [(-5.0, -0.25, 0.135), (-5.0, 0.25, 0.135), (-5.0, -0.25, 0.4), (-5.0, 0.25, 0.4)]
    box_pos = {i: np.array(truck_slots[i]) for i in range(N_BOXES)}
    stage_slots = [(STAGING[0] - 0.2 + 0.4 * (i % 2), STAGING[1], 0.265 + 0.25 * (i // 2)) for i in range(N_BOXES)]
    unload = {"i": 0, "t0": 3.0}
    obstacle_on, obstacle_seen, dropped = False, False, False
    phases = {"Atlas": 0.0, "Spot": 0.0}

    fps, W, H, PANEL = 20, 1280, 720, 330
    if args.video:
        r = mujoco.Renderer(m, H, W - PANEL)
        opt = mujoco.MjvOption()
        opt.geomgroup[:] = 0
        opt.geomgroup[0] = opt.geomgroup[2] = 1
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [-0.7, 0.2, 0.3]
        cam.distance, cam.azimuth, cam.elevation = 17.0, 108, -40  # whole cell: trailer (left) to line (right)
        ff = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                               "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", args.video],
                              stdin=subprocess.PIPE)
        try:
            font = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 17)
            small = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 14)
        except OSError:
            font = small = ImageFont.load_default()

    t, dt = 0.0, 1.0 / fps
    while t < args.seconds:
        # --- Stretch: unload one box every 8 s (reach in, pull, swing, place) ---------------------
        k = unload["i"]
        if k < N_BOXES:
            s = (t - unload["t0"]) / 8.0
            src, dst = box_pos[k].copy(), np.array(stage_slots[k])
            if s < 0:
                face = np.array(truck_slots[k]) + [0.18, 0, 0]
                stretch.stage = "waiting"
            elif s < 0.35:
                face = np.array(truck_slots[k]) + [0.18, 0, 0]
                stretch.stage = f"picking box {k + 1}"
            elif s < 1.0:
                u = (s - 0.35) / 0.65
                u = 0.5 - 0.5 * math.cos(math.pi * u)
                lift = np.array([0, 0, 0.5 * math.sin(math.pi * u)])
                pa, pb = np.array(truck_slots[k]), dst
                box_pos[k] = pa + (pb - pa) * u + lift
                face = box_pos[k] + [0.18, 0, 0]
                stretch.stage = f"placing box {k + 1}"
            else:
                box_pos[k] = dst
                fm.box_staged(t, k + 1)
                unload["i"] += 1
                unload["t0"] = t + 0.5
                face = np.array(truck_slots[min(k + 1, N_BOXES - 1)]) + [0.18, 0, 0]
        else:
            stretch.stage = "trailer empty"
            face = np.array([-4.6, 0.0, 1.0])
        # --- world events -----------------------------------------------------------------------
        if t >= 30 and not obstacle_on:
            obstacle_on = True
            fm.say(t, "pallet left in the aisle")
        if obstacle_on and not obstacle_seen and math.dist(spot.pos, OBSTACLE) < 3.0:
            obstacle_seen = True
            fm.obstacle_seen(t, (OBSTACLE[0] - 0.5, -0.45, OBSTACLE[0] + 0.5, 0.45), "Spot")
        if t >= 55 and not dropped and atlas.carrying is not None and atlas.path:
            dropped = True
            box = atlas.carrying
            box_pos[box - 1] = np.array([atlas.pos[0], atlas.pos[1], 0.125])
            fm.box_dropped(t, "Atlas", box, [atlas.pos[0] + 0.3, atlas.pos[1]])
            atlas.task, atlas.path, atlas.stage = None, [], "idle"
        # --- fleet --------------------------------------------------------------------------------
        before = {n: list(r.pos) for n, r in fm.robots.items()}
        fm.tick(t, dt)
        for name in phases:
            rr = fm.robots[name]
            phases[name] += math.dist(before[name], rr.pos) / (0.55 if name == "Atlas" else 0.4) * math.pi
        # --- pose everything ----------------------------------------------------------------------
        P.free("obstacle", (*OBSTACLE, 0.0) if obstacle_on else (0, 0, -5))
        P.q("stretch/base_x", STRETCH_POS[0])
        P.q("stretch/base_y", STRETCH_POS[1])
        P.q("stretch/base_yaw", STRETCH_YAW)
        P.atlas(*atlas.pos, atlas.heading, phases["Atlas"], bool(atlas.path), atlas.carrying is not None)
        P.spot(*spot.pos, spot.heading, phases["Spot"], bool(spot.path))
        mujoco.mj_kinematics(m, d)
        P.stretch_arm(face)
        if atlas.carrying is not None:
            h = atlas.heading
            box_pos[atlas.carrying - 1] = np.array([atlas.pos[0] + 0.42 * math.cos(h),
                                                    atlas.pos[1] + 0.42 * math.sin(h), 1.05])
        for box in fm.delivered:
            box_pos[box - 1] = np.array([LINE[0], -0.9 + 0.45 * fm.delivered.index(box), 1.07])
        for i in range(N_BOXES):
            P.free(f"box{i}", box_pos[i], quat_yaw(atlas.heading) if atlas.carrying == i + 1 else (1, 0, 0, 0))
        mujoco.mj_forward(m, d)

        if args.video:
            r.update_scene(d, cam, opt)
            frame = Image.new("RGB", (W, H), (18, 19, 22))
            frame.paste(Image.fromarray(r.render()), (0, 0))
            dr = ImageDraw.Draw(frame)
            x0 = W - PANEL + 16
            dr.text((x0, 14), f"Fleet manager   t = {t:5.1f} s", font=font, fill=(240, 240, 240))
            dr.text((x0, 40), f"delivered {len(fm.delivered)}/{N_BOXES}   staged {len(fm.staged)}", font=small,
                    fill=(180, 220, 180))
            y = 72
            for rr in fm.robots.values():
                color = (255, 110, 110) if rr.fault else (255, 210, 90) if rr.stage not in ("idle", "waiting") else (170, 170, 170)
                dr.text((x0, y), rr.name, font=font, fill=color)
                dr.text((x0 + 80, y + 2), rr.stage, font=small, fill=(220, 220, 220))
                y += 28
            dr.text((x0, y + 10), "Events", font=font, fill=(240, 240, 240))
            y += 38
            for when, msg in fm.log[-14:]:
                dr.text((x0, y), f"{when:5.1f}  {msg}"[:44], font=small, fill=(200, 200, 205))
                y += 20
            ff.stdin.write(frame.tobytes())
        t = round(t + dt, 4)
        if len(fm.delivered) == N_BOXES and t > fm.log[-1][0] + 4:
            break

    if args.video:
        ff.stdin.close()
        ff.wait()
    for when, msg in fm.log:
        print(f"{when:6.1f}s  {msg}")
    print(f"delivered {sorted(fm.delivered)} in {t:.0f} s")
    assert sorted(fm.delivered) == list(range(1, N_BOXES + 1)), "not every box reached the line"


if __name__ == "__main__":
    main()
