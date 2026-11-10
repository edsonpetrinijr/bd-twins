"""CAD parts (build123d) for the visual shell. Writes STL files to atlas/meshes/.

Run: python atlas/cad.py && python atlas/build.py
Shapes follow the BD photos/renders listed in docs/atlas-sources.md. Units: meters.
Frames match the body each part attaches to (x forward, y left, z up, joint at the origin).
"""

from pathlib import Path

from build123d import (
    Axis, BuildPart, BuildSketch, Circle, Cylinder, Locations, Plane, Mode,
    export_stl, extrude, fillet, make_hull,
)

OUT = Path(__file__).parent / "meshes"
OUT.mkdir(exist_ok=True)


def save(part, name):
    export_stl(part, str(OUT / f"{name}.stl"), tolerance=0.0004, angular_tolerance=0.08)
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


if __name__ == "__main__":
    save(paddle(length=0.336, r_top=0.066, r_bot=0.058, thick=0.05, edge=0.014), "shin_paddle")
    save(drum(r=0.062, width=0.07, hub_r=0.03, hub_depth=0.012, edge=0.008), "knee_drum")
    save(drum(r=0.056, width=0.06, hub_r=0.028, hub_depth=0.011, edge=0.007), "ankle_drum")
