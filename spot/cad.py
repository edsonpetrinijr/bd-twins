"""CAD parts for Spot (build123d) -> spot/meshes/*.stl. Shapes drawn from BD photos/videos; sizes in meters.
Reuses the shape helpers from atlas/cad.py. Run: python spot/cad.py && python spot/build.py
"""

import sys
from pathlib import Path

from build123d import (Axis, Box, BuildPart, BuildSketch, Cylinder, Locations, Mode, Plane, Polygon,
                       export_stl, extrude, fillet)

sys.path.insert(0, str(Path(__file__).parent.parent / "atlas"))
from cad import drum, rbox, tube  # noqa: E402

OUT = Path(__file__).parent / "meshes"
OUT.mkdir(exist_ok=True)


def save(part, name):
    export_stl(part, str(OUT / f"{name}.stl"), tolerance=0.001, angular_tolerance=0.15)
    print("wrote", name)


def body_shell():
    """Yellow body: side profile with a flat top and both ends sloping down to the camera faces,
    extruded across the width, soft edges. Front/rear face recesses are cut in."""
    L, top, bot, nose = 0.44, 0.065, -0.075, 0.41
    with BuildPart() as p:
        with BuildSketch(Plane.XZ):  # ends lean back ~15 deg above the middle, then tuck under
            Polygon((-nose, top), (nose, top), (L, -0.02), (L - 0.025, bot), (-L + 0.025, bot), (-L, -0.02),
                    align=None)
        extrude(amount=0.12, both=True)
        fillet(p.edges().filter_by(Axis.X), 0.02)   # long edges first
        fillet(p.edges().filter_by(Axis.Y), 0.012)
    return p.part


def side_details():
    """Black details on the body: side stereo-camera slots front and rear, payload rails on top."""
    parts = [rbox(0.075, 0.012, 0.03, 0.004, at=(x, y, 0.005)) for x in (0.27, -0.27) for y in (0.121, -0.121)]
    parts += [rbox(0.5, 0.018, 0.012, 0.004, at=(0, y, 0.071)) for y in (0.075, -0.075)]
    parts += [rbox(0.56, 0.248, 0.01, 0.004, at=(0, 0, -0.005))]                     # side groove line
    parts += [rbox(0.16, 0.25, 0.1, 0.02, at=(x, 0, -0.02)) for x in (0.3, -0.3)]      # black hip recesses at the corners
    out = parts[0]
    for q in parts[1:]:
        out = out + q
    return out


def body_core():
    """Black underside/frame between the legs."""
    return rbox(0.8, 0.246, 0.07, 0.02, at=(0, 0, -0.045))  # black lower half of the body sides


def face(side):
    """Camera face on the front (side=+1) or rear (-1): large dark panel inside a thick yellow bezel,
    leaning back with the end of the body; mesh grille in the middle, camera pods on one side."""
    import math
    tilt = math.degrees(math.atan2(0.03, 0.085))
    panel = rbox(0.012, 0.17, 0.07, 0.005)
    pods = [rbox(0.008, 0.045, 0.02, 0.003, at=(0.009, 0.05, z)) for z in (0.02, 0.0, -0.02)]
    with BuildPart() as lens:
        for z in (0.02, 0.0, -0.02):
            with Locations((0.014, 0.05, z)):
                Cylinder(0.006, 0.004, rotation=(0, 90, 0))
    f = panel
    for q in pods:
        f = f + q
    f = (f + lens.part).rotate(Axis.Y, tilt)
    if side < 0:
        f = f.rotate(Axis.Z, 180)
    return f.translate((side * 0.428, 0, 0.022))


def hip_motor():
    """Black hip housing outside the body corner (left side; mirrored for the right). The thigh hangs out of it."""
    return rbox(0.15, 0.11, 0.1, 0.02, at=(0, 0.08, -0.005))


def upper_leg():
    """Thigh: wide yellow outer cover (deep front-back, thin sideways) over a black inner frame,
    pivot at z=0, knee at (0.025, -0.3205)."""
    cover = tube(-0.02, -0.29, (0.15, 0.066), (0.1, 0.052), 0.022, 0.016)  # top tucked under the hip housing
    frame = tube(0.02, -0.31, (0.09, 0.05), (0.06, 0.045), 0.015, 0.008)
    return cover, frame


def lower_leg():
    """Shin: black, thick at the knee linkage, tapering to a slim rod; rubber foot."""
    shin = tube(0.03, -0.31, (0.07, 0.045), (0.028, 0.026), 0.012, 0.008)
    with BuildPart() as ft:
        with Locations((0, 0, -0.3365)):
            Cylinder(0.036, 0.045, rotation=(90, 0, 0))
        fillet(ft.edges(), 0.018)
    knee = drum(r=0.034, width=0.05, hub_r=0.014, hub_depth=0.006, edge=0.005)
    hook = rbox(0.03, 0.028, 0.08, 0.008, at=(0.02, 0, -0.29), rot=(0, -28, 0))  # forward hook above the foot
    return shin + hook, ft.part, knee


if __name__ == "__main__":
    save(body_shell(), "body_shell")
    save(side_details(), "side_details")
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
