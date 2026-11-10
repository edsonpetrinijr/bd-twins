"""CAD parts (build123d) for the visual shell. Writes STL files to atlas/meshes/.

Run: python atlas/cad.py && python atlas/build.py
Shapes follow the BD photos/renders listed in docs/atlas-sources.md. Units: meters.
Frames match the body each part attaches to (x forward, y left, z up, joint at the origin).
"""

from pathlib import Path

from build123d import (
    Axis, Box, BuildPart, BuildSketch, Circle, Cylinder, Locations, Mode, Plane, RectangleRounded, Rot,
    Polyline, Rectangle, Transition, export_stl, extrude, fillet, loft, make_hull, sweep, BuildLine,
)

OUT = Path(__file__).parent / "meshes"
OUT.mkdir(exist_ok=True)


def save(part, name):
    export_stl(part, str(OUT / f"{name}.stl"), tolerance=0.001, angular_tolerance=0.15)
    print("wrote", name)


def paddle(length, r_top, r_bot, thick, edge):
    """Shin slab: stadium seen from the side (round ends on the knee and ankle axes), soft edges."""
    with BuildPart() as p:
        with BuildSketch(Plane.XZ) as sk:
            with Locations((0, 0)):
                Circle(r_top)
            with Locations((0, -length)):
                Circle(r_bot)
            make_hull(sk.edges())
        extrude(amount=thick / 2, both=True)
        ys = p.edges().group_by(Axis.Y)
        fillet(ys[0] + ys[-1], edge)
    return p.part


def drum(r, width, hub_r, hub_depth, edge):
    """Actuator drum, axis y, outer face at +y with a recessed hub."""
    with BuildPart() as p:
        Cylinder(r, width, rotation=(90, 0, 0))
        fillet(p.edges(), edge)
        with Locations((0, width / 2 - hub_depth / 2 + 0.0005, 0)):
            Cylinder(hub_r, hub_depth, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    return p.part


def tube(z_top, z_bot, top, bot, corner, cap_fillet):
    """Tapered rounded-rectangle tube (lofted), full width x depth: top=(dx, dy) at z_top, bot at z_bot."""
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(z_top)):
            RectangleRounded(top[0], top[1], corner)
        with BuildSketch(Plane.XY.offset(z_bot)):
            RectangleRounded(bot[0], bot[1], corner)
        loft()
        zs = p.edges().group_by(Axis.Z)
        fillet(zs[0] + zs[-1], cap_fillet)
    return p.part


def ribbed(r, height, ribs, groove_depth, axis_y=False):
    """Motor housing with ring grooves (the black finned sections on hips, thighs and elbows)."""
    rot = (90, 0, 0) if axis_y else (0, 0, 0)
    with BuildPart() as p:
        Cylinder(r, height, rotation=rot)
        fillet(p.edges(), r * 0.15)
        pitch = height / (ribs + 1)
        for i in range(1, ribs + 1):
            off = -height / 2 + i * pitch
            loc = (0, off, 0) if axis_y else (0, 0, off)
            with Locations(loc):
                Cylinder(r + 0.01, pitch * 0.4, rotation=rot, mode=Mode.SUBTRACT)
            with Locations(loc):
                Cylinder(r - groove_depth, pitch * 0.4, rotation=rot)
    return p.part


def dome(z_bot, z_top, bot, top, corner, top_fillet):
    """Torso yoke: lofted block whose top edges get a large fillet (the shoulder dome)."""
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(z_bot)):
            RectangleRounded(bot[0], bot[1], corner)
        with BuildSketch(Plane.XY.offset(z_top)):
            RectangleRounded(top[0], top[1], corner)
        loft()
        zs = p.edges().group_by(Axis.Z)
        fillet(zs[-1], top_fillet)
        fillet(p.edges().group_by(Axis.Z)[0], 0.01)
    return p.part


def head():
    """Head: cylinder along x with soft back edge, recessed front face, light ring proud of the shell."""
    with BuildPart() as shell:
        Cylinder(0.1, 0.22, rotation=(0, 90, 0))
        back = shell.edges().sort_by(Axis.X)[0]
        fillet([back], 0.03)
        with Locations((0.11 - 0.012, 0, 0)):
            Cylinder(0.088, 0.03, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    with BuildPart() as ring:
        with Locations((0.112, 0, 0)):
            Cylinder(0.101, 0.024, rotation=(0, 90, 0))
            Cylinder(0.084, 0.03, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
        fillet(ring.edges().sort_by(Axis.X)[-2:], 0.004)
    with BuildPart() as face:
        with Locations((0.101, 0, 0)):
            Cylinder(0.086, 0.008, rotation=(0, 90, 0))
    return shell.part, ring.part, face.part


def rbox(dx, dy, dz, r, at=(0, 0, 0), rot=(0, 0, 0)):
    """Box with all edges rounded, centered at `at`."""
    with BuildPart() as p:
        Box(dx, dy, dz)
        fillet(p.edges(), r)
    return p.part.rotate(Axis.X, rot[0]).rotate(Axis.Y, rot[1]).rotate(Axis.Z, rot[2]).translate(at)


def shoulder():
    """Shoulder actuator: fat drum along x with a soft rim and a recessed front hub."""
    with BuildPart() as p:
        Cylinder(0.074, 0.1, rotation=(0, 90, 0))
        fillet(p.edges(), 0.03)
        with Locations((0.05 - 0.005, 0, 0)):
            Cylinder(0.034, 0.012, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return p.part


def hand(side):
    """Left hand (side=+1) or right (-1): palm, back plate on the outer side, 3 curled fingers, thumb at the front."""
    s = side
    frame = rbox(0.084, 0.044, 0.1, 0.012, at=(0, 0, -0.05))
    for x in (-0.026, 0.0, 0.026):
        z, ang = -0.1, 0.0
        for length in (0.03, 0.024, 0.02):
            ang += 0.18  # slight curl toward the palm
            dz, dy = -length * 0.98, -s * length * 0.18
            seg = rbox(0.02, 0.019, length + 0.006, 0.006, at=(x, -s * 0.004 + dy * (ang / 0.18) * 0.5, z + dz / 2),
                       rot=(s * ang * 57.3, 0, 0))
            frame = frame + seg
            z += dz
    thumb1 = rbox(0.022, 0.02, 0.04, 0.007, at=(0.05, -s * 0.012, -0.045), rot=(0, -35, 0))
    thumb2 = rbox(0.019, 0.018, 0.032, 0.006, at=(0.068, -s * 0.016, -0.078), rot=(0, -10, 0))
    frame = frame + thumb1 + thumb2
    plate = rbox(0.088, 0.012, 0.085, 0.005, at=(0, s * 0.027, -0.055))
    return frame, plate


def foot():
    """Foot: tapered ankle block, sole plate with heel and toe pads."""
    with BuildPart() as blk:
        with BuildSketch(Plane.XY.offset(-0.03)):
            RectangleRounded(0.08, 0.07, 0.015)
        with BuildSketch(Plane.XY.offset(-0.15)):
            RectangleRounded(0.11, 0.09, 0.02)
        loft()
        fillet(blk.edges().group_by(Axis.Z)[-1], 0.012)
    top = rbox(0.27, 0.11, 0.04, 0.012, at=(0.03, 0, -0.155))
    heel = rbox(0.09, 0.124, 0.03, 0.008, at=(-0.085, 0, -0.181))
    toe = rbox(0.13, 0.124, 0.03, 0.008, at=(0.125, 0, -0.181))
    return blk.part + top + heel + toe


def yoke():
    """Upper torso: three lofted sections so the top rolls down into the shoulders (dome, not a box)."""
    with BuildPart() as p:
        for z, (dx, dy), c in ((0.42, (0.28, 0.304), 0.05), (0.53, (0.27, 0.296), 0.05), (0.6, (0.17, 0.18), 0.05)):
            with BuildSketch(Plane.XY.offset(z)):
                RectangleRounded(dx, dy, c)
        loft()
        fillet(p.edges().group_by(Axis.Z)[-1], 0.035)
    return p.part


def battery():
    """Swappable battery module on the back: rounded block with a grip slot and side rails."""
    body = rbox(0.1, 0.24, 0.13, 0.015)
    with BuildPart() as slot:
        with Locations((-0.05, 0, 0.0)):
            Box(0.02, 0.12, 0.022)
    rails = rbox(0.09, 0.012, 0.12, 0.004, at=(0.0, 0.126, 0)) + rbox(0.09, 0.012, 0.12, 0.004, at=(0.0, -0.126, 0))
    return body - slot.part + rails


def handle():
    """Carry handle over the back: one bent tube (U seen from above, dropping to the back)."""
    with BuildPart() as p:
        with BuildLine() as path:
            Polyline((-0.03, 0.12, 0.57), (-0.17, 0.12, 0.57), (-0.17, 0.12, 0.45), (-0.17, -0.12, 0.45),
                     (-0.17, -0.12, 0.57), (-0.03, -0.12, 0.57))
        with BuildSketch(Plane(origin=(-0.03, 0.12, 0.57), z_dir=(-1, 0, 0))):
            Circle(0.013)
        sweep(transition=Transition.ROUND)
    return p.part


if __name__ == "__main__":
    save(paddle(length=0.336, r_top=0.074, r_bot=0.064, thick=0.066, edge=0.018), "shin_paddle")
    save(drum(r=0.07, width=0.08, hub_r=0.034, hub_depth=0.013, edge=0.009), "knee_drum")
    save(drum(r=0.062, width=0.066, hub_r=0.03, hub_depth=0.012, edge=0.008), "ankle_drum")
    save(drum(r=0.074, width=0.1, hub_r=0.034, hub_depth=0.013, edge=0.01), "hip_drum")
    save(drum(r=0.062, width=0.11, hub_r=0.03, hub_depth=0.012, edge=0.009), "elbow_drum")
    save(tube(0.0, -0.42, (0.15, 0.13), (0.13, 0.112), 0.04, 0.018), "thigh_shell")
    save(ribbed(0.045, 0.19, 9, 0.007), "thigh_motor")
    save(tube(-0.085, -0.235, (0.144, 0.132), (0.12, 0.116), 0.035, 0.02), "upper_arm_shell")
    save(ribbed(0.05, 0.08, 4, 0.006), "elbow_ribs")
    save(tube(-0.04, -0.245, (0.1, 0.094), (0.086, 0.082), 0.038, 0.012), "forearm_core")
    save(tube(-0.06, -0.215, (0.108, 0.108), (0.084, 0.092), 0.025, 0.012), "forearm_shell")
    save(tube(0.41, 0.18, (0.28, 0.296), (0.26, 0.272), 0.045, 0.012), "belly")
    save(yoke(), "yoke")
    save(tube(0.19, 0.06, (0.18, 0.22), (0.17, 0.2), 0.03, 0.01), "waist")
    save(ribbed(0.06, 0.15, 6, 0.006, axis_y=True), "pelvis_drum_low")
    save(ribbed(0.055, 0.17, 7, 0.006, axis_y=True), "pelvis_drum_high")
    hs, hr, hf = head()
    save(hs, "head_shell")
    save(hr, "head_ring")
    save(hf, "head_face")
    save(shoulder(), "shoulder")
    for side, tag in ((1, "l"), (-1, "r")):
        hf, hp = hand(side)
        save(hf, f"hand_frame_{tag}")
        save(hp, f"hand_plate_{tag}")
    save(foot(), "foot")
    save(battery(), "battery")
    save(handle(), "handle")
