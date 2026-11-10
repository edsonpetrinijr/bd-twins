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

## v1 visual pass (09/nov)
Compared with frames of BD's "Spot Launch" video (3/4 and side). Changed: body from a box to a side profile with flat top and sloped camera ends; dark camera faces tilted with the slope; black lower half of the body, side groove line, black hip recesses at the four corners, payload rails on top, side camera slots; thighs deeper (0.15 m front-back) and thicker with rounded tops; shins taper from the knee linkage to a slim rod. No logos or lettering reproduced.

## v2 (09/nov)
- Front/rear: ends lean back ~15 deg then tuck under; large dark camera panel inside a thick yellow bezel, grille + three camera pods on one side (was a small strip on a 35 deg wedge, looked odd).
- Hip: black housing outside each body corner; thigh tops tucked under it (thigh no longer a plank next to the body).
- Standing pose hip 0.85 / knee -1.65 rad: thighs lean like in BD videos, body top 0.555 m (spec range 0.52-0.70).
- Hook above each foot (the forward bend of the lower leg).
- Mustard yellow instead of saturated yellow.
- `spot/check.py`: mass 32.70 kg, 10 s standing under joint PD, tilt 1.3 deg, only the four feet touch.
