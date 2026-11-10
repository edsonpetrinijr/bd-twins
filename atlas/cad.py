"""CAD parts (build123d) for the visual shell. Writes STL files to atlas/meshes/.

Run: python atlas/cad.py && python atlas/build.py
Shapes follow the BD photos/renders listed in docs/atlas-sources.md. Units: meters.
Frames match the body each part attaches to (x forward, y left, z up, joint at the origin).
"""

from pathlib import Path

from build123d import (
    Axis, BuildPart, BuildSketch, Circle, Cylinder, Locations, Mode, Plane, RectangleRounded,
    export_stl, extrude, fillet, loft, make_hull,
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


if __name__ == "__main__":
    save(paddle(length=0.336, r_top=0.066, r_bot=0.058, thick=0.05, edge=0.014), "shin_paddle")
    save(drum(r=0.062, width=0.07, hub_r=0.03, hub_depth=0.012, edge=0.008), "knee_drum")
    save(drum(r=0.056, width=0.06, hub_r=0.028, hub_depth=0.011, edge=0.007), "ankle_drum")
    save(drum(r=0.068, width=0.09, hub_r=0.032, hub_depth=0.012, edge=0.009), "hip_drum")
    save(drum(r=0.062, width=0.11, hub_r=0.03, hub_depth=0.012, edge=0.009), "elbow_drum")
    save(tube(0.0, -0.42, (0.12, 0.096), (0.104, 0.088), 0.03, 0.015), "thigh_shell")
    save(ribbed(0.036, 0.18, 9, 0.006), "thigh_motor")
    save(tube(-0.06, -0.23, (0.144, 0.132), (0.12, 0.116), 0.035, 0.02), "upper_arm_shell")
    save(ribbed(0.05, 0.08, 4, 0.006), "elbow_ribs")
    save(tube(-0.04, -0.245, (0.1, 0.094), (0.086, 0.082), 0.038, 0.012), "forearm_core")
    save(tube(-0.06, -0.215, (0.108, 0.108), (0.084, 0.092), 0.025, 0.012), "forearm_shell")
    save(tube(0.41, 0.18, (0.28, 0.296), (0.26, 0.272), 0.045, 0.012), "belly")
    save(dome(0.42, 0.60, (0.28, 0.304), (0.25, 0.28), 0.05, 0.045), "yoke")
    save(tube(0.19, 0.06, (0.18, 0.22), (0.17, 0.2), 0.03, 0.01), "waist")
    save(ribbed(0.06, 0.15, 6, 0.006, axis_y=True), "pelvis_drum_low")
    save(ribbed(0.055, 0.17, 7, 0.006, axis_y=True), "pelvis_drum_high")
    hs, hr, hf = head()
    save(hs, "head_shell")
    save(hr, "head_ring")
    save(hf, "head_face")
