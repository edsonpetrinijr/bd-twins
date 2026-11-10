# Stretch v0: where each number comes from

## Public (BD Stretch brochure, 2026)
Footprint 40 x 48 in (1.0 x 1.25 m) · 1,300 kg (2,866 lb; an older listing says 1,500) · max reach vertical 3.2 m / horizontal 1.95 m · cases up to 23 kg · battery up to 16 h · 600–800 cases/h (Gen 2 claims 1,000).

## Our estimates (no public kinematics exist)
From the brochure photos (side/3-4 of the robot in a warehouse aisle and in a trailer):
- Base: black chassis with a white band, top deck at 0.60 m, corner lights. Modeled as a planar joint (x, y, yaw) driven directly: the real base is holonomic and never tips or steps, and the fleet sim only needs it to go where it is told.
- Turntable (yaw) 0.72 m diameter, 0.30 m tall, 0.10 m behind the base center. Perception mast 1.3 m on the turntable edge, top at ~2.2 m, light band and camera window.
- Arm: shoulder pitch at 1.30 m, upper arm 0.75 m, forearm 0.70 m, wrist 0.16 m, suction pad 0.42 x 0.58 m with a 6 x 8 cup grid. Lengths chosen so the pad center reaches exactly the brochure's 3.2 m up and 1.95 m out (`stretch/check.py`).
- Joints: turntable yaw, shoulder pitch, elbow pitch, forearm roll, wrist pitch, wrist yaw, pad roll (7, matching the "7 DoF" usually reported; layout is ours).
- Mass split (total 1,300 kg): base 1,020 (batteries, counterweight), turntable + mast 160, arm 60, wrist 10, pad 30.
- No logos or lettering reproduced.
