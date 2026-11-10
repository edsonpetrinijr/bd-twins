"""CAD parts for Stretch (build123d) -> stretch/meshes/*.stl. Shapes drawn from the BD 2026 brochure photos.
Reuses helpers from atlas/cad.py. Run: python stretch/cad.py && python stretch/build.py
"""

import sys
from pathlib import Path

from build123d import Axis, BuildPart, Cylinder, Locations, Mode, export_stl, fillet

sys.path.insert(0, str(Path(__file__).parent.parent / "atlas"))
from cad import drum, rbox, tube  # noqa: E402

OUT = Path(__file__).parent / "meshes"
OUT.mkdir(exist_ok=True)


def save(part, name):
    export_stl(part, str(OUT / f"{name}.stl"), tolerance=0.002, angular_tolerance=0.15)
    print("wrote", name)


def union(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def base_lower():
    """Black chassis (holds batteries and counterweight), 1.25 x 1.0 m footprint, with a darker plinth."""
    return union([rbox(1.25, 1.0, 0.34, 0.06, at=(0, 0, 0.21)),
                  rbox(1.18, 0.93, 0.05, 0.02, at=(0, 0, 0.035))])


def base_skirt():
    """White/grey band around the upper base, slightly proud of the chassis."""
    with BuildPart() as p:
        with Locations((0, 0, 0.47)):
            from build123d import Box
            Box(1.24, 0.99, 0.18)
        fillet(p.edges().filter_by(Axis.Z), 0.07)
        fillet(p.edges().group_by(Axis.Z)[-1], 0.03)
    return p.part


def base_details():
    """Front and rear light bars, corner lights, top deck."""
    bars = [rbox(0.02, 0.5, 0.025, 0.006, at=(s * 0.626, 0, 0.3)) for s in (1, -1)]
    corners = [rbox(0.03, 0.06, 0.03, 0.008, at=(sx * 0.6, sy * 0.47, 0.32)) for sx in (1, -1) for sy in (1, -1)]
    deck = rbox(1.15, 0.9, 0.04, 0.015, at=(0, 0, 0.575))
    return union(bars + [deck]), union(corners)


def turntable():
    """Round turntable on the base: white drum with a dark band at the bottom."""
    with BuildPart() as p:
        with Locations((0, 0, 0.15)):
            Cylinder(0.36, 0.3)
        fillet(p.edges().group_by(Axis.Z)[-1], 0.04)
    with BuildPart() as band:
        with Locations((0, 0, 0.03)):
            Cylinder(0.365, 0.06)
    return p.part, band.part


def mast():
    """Perception mast: slim black column with a light band and a camera window near the top."""
    col = rbox(0.13, 0.18, 1.3, 0.03, at=(0, 0, 0.65))
    light = rbox(0.135, 0.185, 0.03, 0.01, at=(0, 0, 1.18))
    window = rbox(0.01, 0.12, 0.2, 0.004, at=(0.066, 0, 1.0))
    return col, light, window


def shoulder_housing():
    """Black housing on the turntable that carries the first arm joint."""
    return union([rbox(0.3, 0.26, 0.34, 0.05, at=(0, 0, 0.17)),
                  drum(r=0.12, width=0.26, hub_r=0.05, hub_depth=0.015, edge=0.015).translate((0, 0, 0.40))])


def arm_link(length, w0, w1):
    """White arm link along -z from the joint, with a black joint drum at the far end."""
    shell = tube(-0.08, -(length - 0.08), (w0, w0 * 0.9), (w1, w1 * 0.9), w1 * 0.3, 0.02)
    joint = drum(r=w1 * 0.62, width=w1 * 1.15, hub_r=w1 * 0.25, hub_depth=0.01, edge=0.01).translate((0, 0, -length))
    return shell, joint


def wrist():
    """Black wrist block between the forearm and the suction pad."""
    return union([rbox(0.12, 0.12, 0.16, 0.03, at=(0, 0, -0.08)),
                  drum(r=0.06, width=0.13, hub_r=0.025, hub_depth=0.008, edge=0.008).translate((0, 0, -0.16))])


def suction_pad():
    """Vacuum gripper: black plate with a 6 x 8 grid of suction cups on its +x (working) face."""
    plate = rbox(0.06, 0.42, 0.58, 0.012, at=(0, 0, -0.29))
    with BuildPart() as cups:
        for iy in range(6):
            for iz in range(8):
                with Locations((0.045, -0.175 + iy * 0.07, -0.045 - iz * 0.07)):
                    Cylinder(0.024, 0.03, rotation=(0, 90, 0))
                    Cylinder(0.015, 0.03, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return plate, cups.part


if __name__ == "__main__":
    save(base_lower(), "base_lower")
    save(base_skirt(), "base_skirt")
    deck, lights = base_details()
    save(deck, "base_deck")
    save(lights, "corner_lights")
    tt, band = turntable()
    save(tt, "turntable")
    save(band, "turntable_band")
    col, light, window = mast()
    save(col, "mast")
    save(light, "mast_light")
    save(window, "mast_window")
    save(shoulder_housing(), "shoulder_housing")
    for name, (length, w0, w1) in {"upper_arm": (0.75, 0.2, 0.16), "forearm": (0.70, 0.15, 0.12)}.items():
        shell, joint = arm_link(length, w0, w1)
        save(shell, f"{name}_shell")
        save(joint, f"{name}_joint")
    save(wrist(), "wrist")
    plate, cups = suction_pad()
    save(plate, "pad_plate")
    save(cups, "pad_cups")
