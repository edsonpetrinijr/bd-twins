"""CAD parts for Spot (build123d) -> spot/meshes/*.stl. Shapes drawn from BD photos/videos; sizes in meters.
Reuses the shape helpers from atlas/cad.py. Run: python spot/cad.py && python spot/build.py
"""

import sys
from pathlib import Path

from build123d import Axis, Box, BuildPart, Cylinder, Locations, Mode, export_stl, fillet

sys.path.insert(0, str(Path(__file__).parent.parent / "atlas"))
from cad import drum, rbox, tube  # noqa: E402

OUT = Path(__file__).parent / "meshes"
OUT.mkdir(exist_ok=True)


def save(part, name):
    export_stl(part, str(OUT / f"{name}.stl"), tolerance=0.001, angular_tolerance=0.15)
    print("wrote", name)


def body_shell():
    """Yellow body cover: long rounded box, front and rear ends sloped down (the 'faces' with cameras)."""
    with BuildPart() as p:
        Box(0.84, 0.235, 0.15)
        fillet(p.edges().filter_by(Axis.X), 0.035)
        fillet(p.edges().filter_by(Axis.Y), 0.03)
        for x, s in ((0.42, 1), (-0.42, -1)):  # chamfer the ends: cut a tilted block off the lower half
            with Locations((x + s * 0.03, 0, -0.07)):
                Box(0.12, 0.3, 0.1, rotation=(0, s * 35, 0), mode=Mode.SUBTRACT)
    return p.part


def body_core():
    """Black underside/frame between the legs."""
    return rbox(0.62, 0.2, 0.06, 0.02, at=(0, 0, -0.07))


def face(side):
    """Camera bezel on the front (side=+1) or rear (-1): recessed dark panel with two lens pods."""
    panel = rbox(0.02, 0.17, 0.085, 0.008, at=(side * 0.452, 0, -0.012))
    with BuildPart() as lens:
        for y in (-0.045, 0.045):
            with Locations((side * 0.463, y, -0.012)):
                Cylinder(0.014, 0.008, rotation=(0, 90, 0))
    return panel + lens.part


def hip_motor():
    """Hip abduction actuator: black drum along x, hub at the front."""
    return drum(r=0.055, width=0.11, hub_r=0.025, hub_depth=0.01, edge=0.008).rotate(Axis.Z, 90)


def upper_leg():
    """Thigh: yellow cover over a black motor, tapering toward the knee (pivot at z=0, knee at z=-0.3205)."""
    cover = tube(0.04, -0.27, (0.11, 0.06), (0.075, 0.05), 0.022, 0.012)
    motor = drum(r=0.05, width=0.075, hub_r=0.022, hub_depth=0.01, edge=0.007)
    return cover, motor


def lower_leg():
    """Shin: slim black tube from the knee to the foot (z=0 to z=-0.3365), rubber foot sphere at the end."""
    shin = tube(0.02, -0.31, (0.045, 0.035), (0.028, 0.028), 0.012, 0.006)
    with BuildPart() as ft:
        with Locations((0, 0, -0.3365)):
            Cylinder(0.036, 0.05, rotation=(90, 0, 0))
        fillet(ft.edges(), 0.02)
    knee = drum(r=0.032, width=0.05, hub_r=0.014, hub_depth=0.006, edge=0.005)
    return shin, ft.part, knee


if __name__ == "__main__":
    save(body_shell(), "body_shell")
    save(body_core(), "body_core")
    save(face(1), "face_front")
    save(face(-1), "face_rear")
    save(hip_motor(), "hip_motor")
    cover, motor = upper_leg()
    save(cover, "upper_cover")
    save(motor, "upper_motor")
    shin, ft, knee = lower_leg()
    save(shin, "shin")
    save(ft, "foot")
    save(knee, "knee")
