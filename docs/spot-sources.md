# Spot v0: where each number comes from

## Public spec (BD spot-specifications.pdf)
32.7 kg with battery · battery 5.2 kg (324 x 168 x 93 mm) · 1100 x 500 mm · walking height 520–700 mm (default 610) · sitting 191 mm · payload 14 kg · 1.6 m/s · 30 deg slopes · 300 mm steps.

## Joint layout and leg masses (RAI Institute `spot_description`, MIT license)
Numbers only; meshes and code here are ours.
- Hip abduction (x axis) at (±0.29785, ±0.055, 0) from the body center, range ±0.785 rad, 45 Nm.
- Hip pitch (y axis) 0.110945 m further out, range -0.899..2.295 rad, 45 Nm.
- Knee (y axis) at (0.025, 0, -0.3205) from the hip pitch, range -2.793..-0.255 rad, 115 Nm.
- Foot center 0.3365 m below the knee, rubber foot radius 0.036 m.
- Per leg: hip 1.68 kg, upper leg 2.34 kg, lower leg 0.35 kg. Body = 32.7 − 4 × 4.37 = 15.22 kg (includes the battery).

## Our choices
- Standing pose hip pitch 0.75, knee −1.5 rad → body top 0.599 m (spec default walking height 0.61 m).
- Visual shell in `spot/cad.py` (build123d), from the BD photos and the "Spot Launch" video: yellow body with sloped ends and dark camera faces front and rear, black hip actuators, yellow thigh covers, slim black shins, rubber feet.
- Standing test (joint PD only, kp 300–500, implicitfast): 10 s, tilt < 2 deg, body settles 1–2 cm, only the four feet touch the ground.
