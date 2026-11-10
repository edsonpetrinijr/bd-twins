"""Fleet demo ("fake" robots: posed kinematically, no leg physics) with real box physics.

Two trailers, two Stretch unloading onto staging pallets, two Atlas carrying boxes to a conveyor, two Spot
patrolling. The conveyor is a MuJoCo 4 surface-velocity belt (`surfacevel`): delivered boxes ride it under
physics. Mid-run an aisle gets blocked (a Spot sees it, routes are replanned) and an Atlas drops a box
(it falls under physics; a recover task goes first in the queue).

Needs MuJoCo >= 4 (separate env from mjlab training):
  uv venv .venv-fleet --python 3.12 && uv pip install --python .venv-fleet "mujoco>=4" pillow numpy
  .venv-fleet/bin/python fleet/run.py                  # headless: event log + checks
  .venv-fleet/bin/python fleet/run.py --video out.mp4  # plus video with the fleet panel
  .venv-fleet/bin/python fleet/run.py --video out.mp4 --filament  # same, MuJoCo 4 Filament renderer (PBR)
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

BOXES_PER_TRUCK = 4
DOCKS = [  # one trailer + one Stretch + one staging pallet per dock
    {"truck": (-6.0, 1.75), "stretch": (-3.6, 1.75), "staging": (-3.2, 3.5), "pickup": (-2.0, 3.5)},
    {"truck": (-6.0, -1.75), "stretch": (-3.6, -1.75), "staging": (-3.2, -3.5), "pickup": (-2.0, -3.5)},
]
BELT = {"x": 6.6, "y0": -2.6, "y1": 2.6, "z": 0.92, "speed": 0.35}
DROPOFFS = [(5.25, -1.3), (5.25, 1.3)]  # outside the conveyor keep-out (belt edge 6.2 m - 0.35 inflate)
OBSTACLE = (2.0, 0.0)
PATROLS = [[(5.0, 4.1), (-1.0, 4.3), (-1.0, 2.6), (5.0, 2.6)],
           [(4.8, 0.0), (-0.8, 0.0), (-1.0, -4.2), (5.0, -4.2)]]
BOX = (0.35, 0.3, 0.25)
CARD = [0.72, 0.55, 0.36, 1]


def quat_yaw(yaw):
    return [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


class FilamentView:
    """MuJoCo 4 Filament renderer with the same update_scene/render calls as mujoco.Renderer."""

    ctx = None

    def __init__(self, m, h, w):
        from mujoco import mjrf
        import mujoco.studio.window  # noqa: F401  registers the resource providers Filament needs
        from mujoco.rendering.filament.model_scene import ModelScene
        from mujoco.rendering.filament.renderer import Renderer, View
        if FilamentView.ctx is None:  # one GL context per process
            FilamentView.ctx = mjrf.Context(mjrf.ContextConfig())
        self.m, self.View = m, View
        # Filament uses physical units (lux) and ignores MJCF diffuse levels: key light 1e5, fills 3e4
        m.light_intensity[:] = 3e4
        m.light_intensity[-1] = 1e5
        self.r = Renderer(FilamentView.ctx)
        self.ms = ModelScene(FilamentView.ctx, m, self.r.scene("main"))
        for g in range(m.ngeom):  # only visual groups 0 and 2, like the GL view
            if m.geom_group[g] not in (0, 2):
                self.r.scene("main").remove_renderable(self.ms.get_renderable(mujoco.mjtObj.mjOBJ_GEOM, g))
        self.buf = np.zeros((h, w, 3), np.uint8)
        self.r.target("img", self.buf)
        self.scn = mujoco.MjvScene(m, 20000)

    def update_scene(self, d, cam, opt):
        self.ms.update(d)
        mujoco.mjv_updateScene(self.m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, self.scn)

    def render(self):
        self.r.render([self.View(scene="main", target="img", camera=self.scn.camera[0],
                                 headlight=True, headlight_intensity=0.3)])
        return self.buf.copy()


def build_scene(n_boxes):
    w = stage(mujoco.MjSpec(), floor=(0.42, 0.43, 0.45, 1))
    w.option.timestep = 0.004
    w.visual.global_.fovy = 40
    add = w.worldbody.add_geom
    B = mujoco.mjtGeom.mjGEOM_BOX
    add(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[11, 7, 0.1], pos=(0, 0, -0.001), rgba=[0.33, 0.34, 0.36, 1])
    # warehouse lighting: the studio spots only cover ~2 m, so switch to directional sun + fill + soft ambient
    for light in w.lights:
        light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        light.castshadow = False
    key = w.worldbody.add_light(pos=[0, 0, 12], dir=[-0.35, 0.3, -1], diffuse=[0.55, 0.55, 0.52], castshadow=True)
    key.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    w.visual.headlight.ambient = [0.26, 0.26, 0.28]
    w.visual.headlight.diffuse = [0.2, 0.2, 0.2]
    w.visual.map.shadowclip = 14
    w.visual.quality.shadowsize = 8192
    for dock in DOCKS:  # trailers: floor, side walls, front wall; open roof, see-through walls
        tx, ty = dock["truck"]
        for pos, size in (((tx, ty, 0.02), (1.6, 1.05, 0.02)), ((tx, ty + 1.05, 1.3), (1.6, 0.03, 1.3)),
                          ((tx, ty - 1.05, 1.3), (1.6, 0.03, 1.3)), ((tx - 1.6, ty, 1.3), (0.03, 1.05, 1.3))):
            add(type=B, pos=pos, size=size, rgba=[0.78, 0.8, 0.83, 0.3])
        sx, sy = dock["staging"]
        add(type=B, pos=(sx, sy, 0.07), size=(0.55, 0.45, 0.07), rgba=[0.6, 0.45, 0.25, 1])
    # conveyor: frame + belt with surface velocity along +y (MuJoCo 4)
    L = (BELT["y1"] - BELT["y0"]) / 2
    yc = (BELT["y1"] + BELT["y0"]) / 2
    add(type=B, pos=(BELT["x"], yc, 0.44), size=(0.42, L, 0.44), rgba=[0.2, 0.21, 0.23, 1])
    belt = add(type=B, name="belt", pos=(BELT["x"], yc, BELT["z"] - 0.02), size=(0.4, L, 0.02),
               rgba=[0.08, 0.08, 0.09, 1])
    belt.surfacevel = [0, BELT["speed"], 0, 0, 0, 0]
    for y in (-1.6, 1.6):  # open shelving
        for z in (0.03, 0.55, 1.07):
            add(type=B, pos=(2.0, y, z), size=(2.5, 0.4, 0.03), rgba=[0.2, 0.35, 0.65, 1])
        for px in (-0.45, 4.45):
            for py in (y - 0.37, y + 0.37):
                add(type=B, pos=(px, py, 0.55), size=(0.03, 0.03, 0.55), rgba=[0.55, 0.57, 0.6, 1])
        for k in range(6):
            add(type=B, pos=(0.0 + 0.8 * k, y, 0.26 + 0.52 * (k % 2)), size=(0.25, 0.3, 0.2), rgba=CARD)
    for y in (-5.0, 5.0):  # floor lane markings
        add(type=B, pos=(1.0, y, 0.002), size=(7.5, 0.05, 0.002), rgba=[0.95, 0.75, 0.1, 1])
    obstacle = w.worldbody.add_body(name="obstacle", pos=(0, 0, -5), mocap=True)
    obstacle.add_geom(type=B, size=(0.5, 0.45, 0.4), pos=(0, 0, 0.4), rgba=CARD)
    for i in range(n_boxes):
        b = w.worldbody.add_body(name=f"box{i}", pos=(0, 0, -3 - i))
        b.add_freejoint(name=f"box{i}")
        b.add_geom(type=B, size=[s / 2 for s in BOX], rgba=CARD, mass=8, friction=[0.9, 0.01, 0.001])
    robots = [("atlas", "atlas/atlas.xml", "atlas"), ("atlas2", "atlas/atlas.xml", "atlas"),
              ("spot", "spot/spot.xml", "spot/meshes"), ("spot2", "spot/spot.xml", "spot/meshes"),
              ("stretch", "stretch/stretch.xml", "stretch/meshes"), ("stretch2", "stretch/stretch.xml", "stretch/meshes")]
    for name, xml, meshdir in robots:
        child = mujoco.MjSpec.from_file(str(ROOT / xml))
        child.meshdir = str(ROOT / meshdir)
        for g in child.geoms:  # robots are kinematic props here: boxes must not collide with them
            g.contype = g.conaffinity = 0
        w.worldbody.add_frame().attach_body(child.worldbody.first_body(), f"{name}/", "")
    return w.compile()


class Poser:
    """Writes robot poses into qpos each physics step (kinematic robots in a physics world)."""

    def __init__(self, m, d):
        self.m, self.d = m, d

    def q(self, joint, value):
        self.d.qpos[self.m.joint(joint).qposadr[0]] = value

    def free(self, joint, pos, quat=(1, 0, 0, 0)):
        j = self.m.joint(joint)
        a, v = j.qposadr[0], j.dofadr[0]
        self.d.qpos[a:a + 3], self.d.qpos[a + 3:a + 7] = pos, quat
        self.d.qvel[v:v + 6] = 0

    def atlas(self, p, x, y, yaw, phase, moving, carrying, waist=0.0):
        self.free(f"{p}/floating_base_joint", (x, y, 0.975 + (0.015 * abs(math.sin(phase)) if moving else 0)),
                  quat_yaw(yaw))
        self.q(f"{p}/waist_yaw_joint", waist)  # Atlas turns its torso on the legs (continuous waist)
        self.q(f"{p}/neck_yaw_joint", 0.0)
        for side, s in (("left", 1), ("right", -1)):
            off = 0 if s > 0 else math.pi
            swing = 0.35 * math.sin(phase + off) if moving else 0.0
            knee = 0.5 + (0.45 * max(0.0, math.sin(phase + off)) if moving else 0.0)
            self.q(f"{p}/{side}_hip_pitch_joint", -knee / 2 - swing)
            self.q(f"{p}/{side}_knee_joint", knee)
            self.q(f"{p}/{side}_ankle_pitch_joint", -knee / 2 + swing * 0.5)
            if carrying:  # forearms under the box, hands on its sides
                self.q(f"{p}/{side}_shoulder_pitch_joint", -0.55)
                self.q(f"{p}/{side}_shoulder_roll_joint", s * 0.05)
                self.q(f"{p}/{side}_shoulder_yaw_joint", -s * 0.25)
                self.q(f"{p}/{side}_elbow_joint", -1.35)
            else:
                self.q(f"{p}/{side}_shoulder_pitch_joint", -swing * 0.8)
                self.q(f"{p}/{side}_shoulder_roll_joint", s * 0.08)
                self.q(f"{p}/{side}_shoulder_yaw_joint", 0.0)
                self.q(f"{p}/{side}_elbow_joint", -0.35)

    def spot(self, p, x, y, yaw, phase, moving):
        self.free(f"{p}/floating_base_joint", (x, y, 0.49), quat_yaw(yaw))
        for leg, ph in (("front_left", 0), ("rear_right", 0), ("front_right", math.pi), ("rear_left", math.pi)):
            s = math.sin(phase + ph) if moving else 0.0
            self.q(f"{p}/{leg}_hip_y", 0.85 + 0.25 * s)
            self.q(f"{p}/{leg}_knee", -1.65 - 0.35 * max(0.0, s))

    def stretch_base(self, p, x, y, yaw):
        self.q(f"{p}/base_x", x)
        self.q(f"{p}/base_y", y)
        self.q(f"{p}/base_yaw", yaw)

    def stretch_arm(self, p, target):
        """Turntable yaw + 2-link IK so the vertical pad faces `target` (world xyz of the box face)."""
        m, d = self.m, self.d
        tt = d.xpos[m.body(f"{p}/turntable").id]
        base_yaw = d.qpos[m.joint(f"{p}/base_yaw").qposadr[0]]
        dx, dy = target[0] - tt[0], target[1] - tt[1]
        R = math.hypot(dx, dy) - 0.05 - 0.08
        Z = target[2] + 0.29 + 0.16 - (tt[2] + 0.70)
        L1, L2 = 0.75, 0.70
        D = min(math.hypot(R, Z), L1 + L2 - 1e-3)
        a = math.atan2(Z, R)
        a1 = a + math.acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D))
        e = (D * math.cos(a) - L1 * math.cos(a1), D * math.sin(a) - L1 * math.sin(a1))
        th1 = math.atan2(-L1 * math.cos(a1), -L1 * math.sin(a1))
        th12 = math.atan2(-e[0], -e[1])
        yaw = math.atan2(dy, dx) - base_yaw
        self.q(f"{p}/turntable_yaw", (yaw + math.pi) % (2 * math.pi) - math.pi)
        self.q(f"{p}/shoulder_pitch", th1)
        self.q(f"{p}/elbow_pitch", th12 - th1)
        self.q(f"{p}/wrist_pitch", -th12)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video")
    ap.add_argument("--seconds", type=float, default=150)
    ap.add_argument("--start", type=float, default=0.0, help="video starts here; the sim before it runs unrendered")
    ap.add_argument("--filament", action="store_true", help="render with the MuJoCo 4 Filament renderer")
    ap.add_argument("--frame", type=float, help="save one still at this time (needs --video for the path)")
    args = ap.parse_args()

    n_boxes = BOXES_PER_TRUCK * len(DOCKS)
    m = build_scene(n_boxes)
    d = mujoco.MjData(m)
    P = Poser(m, d)

    grid = GridMap(-8.5, -5.5, 8.5, 5.5)
    for dock in DOCKS:
        tx, ty = dock["truck"]
        grid.block_rect(tx - 1.6, ty - 1.05, tx + 1.6, ty + 1.05)
        sx, sy = dock["stretch"]
        grid.block_rect(sx - 0.65, sy - 0.5, sx + 0.65, sy + 0.5)
        gx, gy = dock["staging"]
        grid.block_rect(gx - 0.55, gy - 0.45, gx + 0.55, gy + 0.45)
    grid.block_rect(BELT["x"] - 0.42, BELT["y0"], BELT["x"] + 0.42, BELT["y1"])
    for y in (-1.6, 1.6):
        grid.block_rect(-0.5, y - 0.4, 4.5, y + 0.4, inflate=0.25)
    atlases = [Robot("Atlas 1", "atlas", [-1.0, 4.4], 1.0, {"carry", "recover"}, home=(-1.0, 4.4)),
               Robot("Atlas 2", "atlas", [-1.0, -4.4], 1.0, {"carry", "recover"}, home=(-1.0, -4.4))]
    spots = [Robot("Spot 1", "spot", [5.0, 4.1], 1.3, {"inspect"}),
             Robot("Spot 2", "spot", [4.8, 0.0], 1.3, {"inspect"})]
    stretches = [Robot(f"Stretch {i + 1}", "stretch", list(dk["stretch"]), 0.0, {"unload"}) for i, dk in enumerate(DOCKS)]
    fm = FleetManager(grid, atlases + spots + stretches, staging=None, line=None, patrol=PATROLS[0])
    fm.patrols = {"Spot 1": PATROLS[0], "Spot 2": PATROLS[1]}
    fm.line = DROPOFFS[0]  # where recovered boxes go
    prefix = {"Atlas 1": "atlas", "Atlas 2": "atlas2", "Spot 1": "spot", "Spot 2": "spot2",
              "Stretch 1": "stretch", "Stretch 2": "stretch2"}

    boxes = {}
    for k, dock in enumerate(DOCKS):
        tx, ty = dock["truck"]
        for j in range(BOXES_PER_TRUCK):
            i = k * BOXES_PER_TRUCK + j
            slot = (tx + 1.0, ty - 0.2 + 0.4 * (j % 2), 0.165 + 0.26 * (j // 2))
            boxes[i] = {"state": "truck", "pos": np.array(slot), "dock": k, "slot": slot}
    unload = [{"next": k * BOXES_PER_TRUCK, "t0": 3.0 + 4.0 * k} for k in range(len(DOCKS))]
    staged_slots = [[] for _ in DOCKS]
    shipped, events_done = [], set()
    phases = {r.name: 0.0 for r in atlases + spots}
    waist = {a.name: 0.0 for a in atlases}

    def torso_target(a):
        """Where Atlas' torso should face: the pallet or belt while handing over, else straight ahead."""
        if a.dwell <= 0 and not (a.goal and a.path and math.dist(a.pos, a.goal) < 1.2):
            return 0.0
        if a.carrying is None and a.task is not None and a.task.kind == "carry":
            k = next(k for k, dk in enumerate(DOCKS) if tuple(dk["pickup"]) == tuple(a.task.src))
            tx, ty = DOCKS[k]["staging"]
        elif a.carrying is not None:
            tx, ty = BELT["x"], a.pos[1]
        else:
            return 0.0
        rel = math.atan2(ty - a.pos[1], tx - a.pos[0]) - a.heading
        return (rel + math.pi) % (2 * math.pi) - math.pi

    def staging_slot(k, n):
        gx, gy = DOCKS[k]["staging"]
        return np.array([gx - 0.2 + 0.4 * (n % 2), gy, 0.265 + 0.26 * (n // 2)])

    fps, W, H, PANEL = 20, 1280, 720, 340
    if args.video:
        View = FilamentView if args.filament else mujoco.Renderer
        r = View(m, H, W - PANEL)
        opt = mujoco.MjvOption()
        opt.geomgroup[:] = 0
        opt.geomgroup[0] = opt.geomgroup[2] = 1
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [-0.6, 0.0, 0.3]
        cam.distance, cam.azimuth, cam.elevation = 17.5, 100, -46
        IW, IH = 400, 225
        inset_r = View(m, IH, IW)
        follow = mujoco.MjvCamera()
        follow.distance, follow.elevation = 3.4, -14
        ff = None if args.frame else subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                               "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", args.video],
                              stdin=subprocess.PIPE)
        try:
            font = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 17)
            small = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 13)
        except OSError:
            font = small = ImageFont.load_default()

    box_dofs = {m.joint(f"box{i}").dofadr[0] + k for i in range(n_boxes) for k in range(6)}
    robot_dofs = np.array([v for v in range(m.nv) if v not in box_dofs])  # kinematic: no velocity build-up

    frame_dt = 1.0 / fps
    substeps = int(round(frame_dt / m.opt.timestep))
    t = 0.0
    while t < args.seconds:
        # --- Stretch: one box every 8 s per dock ------------------------------------------------
        faces = []
        for k, (u, st) in enumerate(zip(unload, stretches)):
            i = u["next"]
            if i < (k + 1) * BOXES_PER_TRUCK:
                b = boxes[i]
                s = (t - u["t0"]) / 8.0
                dst = staging_slot(k, len(staged_slots[k]))
                if s < 0.35:
                    st.stage = "waiting" if s < 0 else f"picking box {i + 1}"
                    faces.append(np.array(b["slot"]) + [0.18, 0, 0])
                elif s < 1.0:
                    v = 0.5 - 0.5 * math.cos(math.pi * (s - 0.35) / 0.65)
                    b["state"] = "stretch"
                    b["pos"] = np.array(b["slot"]) + (dst - np.array(b["slot"])) * v + [0, 0, 0.5 * math.sin(math.pi * v)]
                    st.stage = f"placing box {i + 1}"
                    faces.append(b["pos"] + [0.18, 0, 0])
                else:
                    b["state"], b["pos"] = "staging", dst
                    staged_slots[k].append(i)
                    fm.add_task("carry", src=DOCKS[k]["pickup"], dst=DROPOFFS[i % 2], box=i + 1)
                    fm.staged.append(i + 1)
                    fm.say(t, f"box {i + 1} staged at dock {k + 1} -> carry task")
                    u["next"] += 1
                    u["t0"] = t + 0.5
                    faces.append(dst + [0.18, 0, 0])
            else:
                st.stage = "trailer empty"
                faces.append(np.array([*DOCKS[k]["stretch"], 0]) + [-1.0, 0, 1.0])
        # --- world events -------------------------------------------------------------------------
        if t >= 35 and "obstacle" not in events_done:
            events_done.add("obstacle")
            fm.say(t, "pallet left in the center aisle")
        if "obstacle" in events_done and "seen" not in events_done:
            for sp in spots:
                if math.dist(sp.pos, OBSTACLE) < 3.0:
                    events_done.add("seen")
                    fm.obstacle_seen(t, (OBSTACLE[0] - 0.5, -0.45, OBSTACLE[0] + 0.5, 0.45), sp.name)
        if t >= 60 and "drop" not in events_done:
            for a in atlases:
                if a.carrying is not None and a.path:
                    events_done.add("drop")
                    boxes[a.carrying - 1]["state"] = "free"  # falls under physics from the hands
                    fm.box_dropped(t, a.name, a.carrying, [a.pos[0] + 0.3, a.pos[1]])
                    a.task, a.path, a.stage = None, [], "idle"
                    break
        # --- fleet --------------------------------------------------------------------------------
        before = {rr.name: list(rr.pos) for rr in atlases + spots}
        fm.tick(t, frame_dt)
        for rr in atlases + spots:
            phases[rr.name] += math.dist(before[rr.name], rr.pos) / (0.55 if rr.body == "atlas" else 0.4) * math.pi
        for a in atlases:  # waist turns at up to 2.5 rad/s toward its target
            err = torso_target(a) - waist[a.name]
            waist[a.name] += max(-2.5 * frame_dt, min(2.5 * frame_dt, err))
        for a in atlases:
            if a.carrying is not None and boxes[a.carrying - 1]["state"] != a.name:
                i = a.carrying - 1
                if boxes[i]["state"] == "staging":
                    k = boxes[i]["dock"]
                    staged_slots[k].remove(i)
                    for n, j in enumerate(staged_slots[k]):
                        boxes[j]["pos"] = staging_slot(k, n)
                boxes[i]["state"] = a.name
        for i in list(fm.delivered):
            if boxes[i - 1]["state"] not in ("belt", "shipped"):
                boxes[i - 1]["state"] = "belt"
                dy = DROPOFFS[(i - 1) % 2][1]
                P.free(f"box{i - 1}", (BELT["x"], dy, BELT["z"] + 0.14))
        # --- physics substeps: robots and held boxes are kinematic, free boxes are simulated ------
        for _ in range(substeps):
            for k, st in enumerate(stretches):
                P.stretch_base(prefix[st.name], *DOCKS[k]["stretch"], math.pi)
            for a in atlases:
                P.atlas(prefix[a.name], *a.pos, a.heading, phases[a.name], bool(a.path), a.carrying is not None,
                        waist[a.name])
            for sp in spots:
                P.spot(prefix[sp.name], *sp.pos, sp.heading, phases[sp.name], bool(sp.path))
            for i, b in boxes.items():
                held = next((a for a in atlases if a.name == b["state"]), None)
                if held is not None:
                    h = held.heading + waist[held.name]  # the box is in front of the torso, not the legs
                    b["pos"] = np.array([held.pos[0] + 0.45 * math.cos(h), held.pos[1] + 0.45 * math.sin(h), 1.02])
                    P.free(f"box{i}", b["pos"], quat_yaw(h))
                elif b["state"] in ("truck", "stretch", "staging"):
                    P.free(f"box{i}", b["pos"])
                elif b["state"] == "shipped":
                    P.free(f"box{i}", (40 + i, 40, 0.2))  # parked far outside the camera view
            d.mocap_pos[m.body("obstacle").mocapid[0]] = (*OBSTACLE, 0) if "obstacle" in events_done else (0, 0, -5)
            mujoco.mj_kinematics(m, d)
            for k, st in enumerate(stretches):
                P.stretch_arm(prefix[st.name], faces[k])
            d.qvel[robot_dofs] = 0
            mujoco.mj_step(m, d)
        for i, b in boxes.items():  # boxes riding off the end of the belt are shipped
            if b["state"] == "belt" and d.qpos[m.joint(f"box{i}").qposadr[0] + 1] > BELT["y1"] - 0.2:
                b["state"] = "shipped"
                shipped.append(i + 1)
                fm.say(t, f"box {i + 1} shipped by the conveyor")

        if args.video and t >= (args.start if args.frame is None else args.frame):
            r.update_scene(d, cam, opt)
            frame = Image.new("RGB", (W, H), (16, 17, 20))
            frame.paste(Image.fromarray(r.render()), (0, 0))
            a1 = atlases[0]
            follow.lookat[:] = [a1.pos[0], a1.pos[1], 0.95]
            follow.azimuth = math.degrees(a1.heading) + 145
            inset_r.update_scene(d, follow, opt)
            inset = Image.fromarray(inset_r.render())
            frame.paste((235, 235, 235), (10, H - IH - 14, 14 + IW, H - 10))
            frame.paste(inset, (12, H - IH - 12))
            dr = ImageDraw.Draw(frame)
            dr.text((20, H - IH - 8), "Atlas 1", font=small, fill=(255, 255, 255))
            x0 = W - PANEL + 14
            dr.text((x0, 12), f"Fleet manager    t = {t:5.1f} s", font=font, fill=(240, 240, 240))
            dr.text((x0, 36), f"delivered {len(fm.delivered)}/{n_boxes}   shipped {len(shipped)}   "
                    f"staged {len(fm.staged)}", font=small, fill=(170, 220, 170))
            y = 62
            for rr in atlases + spots + stretches:
                busy = rr.stage not in ("idle", "waiting", "trailer empty", "parking")
                color = (255, 110, 110) if rr.fault else (255, 205, 90) if busy else (165, 165, 170)
                dr.text((x0, y), rr.name, font=small, fill=color)
                dr.text((x0 + 72, y), rr.stage[:30], font=small, fill=(215, 215, 220))
                y += 19
            dr.text((x0, y + 8), "Events", font=font, fill=(240, 240, 240))
            y += 32
            for when, msg in fm.log[-((H - y - 10) // 18):]:
                dr.text((x0, y), f"{when:5.1f}  {msg}"[:46], font=small, fill=(195, 195, 200))
                y += 18
            if args.frame is not None:
                frame.save(args.video)
                return
            ff.stdin.write(frame.tobytes())
        t = round(t + frame_dt, 4)
        if len(shipped) == n_boxes:
            break

    if args.video:
        ff.stdin.close()
        ff.wait()
    for when, msg in fm.log:
        print(f"{when:6.1f}s  {msg}")
    print(f"delivered {len(fm.delivered)}/{n_boxes}, shipped {len(shipped)}/{n_boxes} in {t:.0f} s")
    if args.seconds >= 150:  # a clipped run (--seconds) stops before every box is out
        assert len(shipped) == n_boxes, "not every box left on the conveyor"


if __name__ == "__main__":
    main()
