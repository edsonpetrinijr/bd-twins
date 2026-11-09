# Atlas v0: where each number comes from

Rule: every number in `atlas/atlas.xml` is either from a public source or marked as an estimate.

## Public (Boston Dynamics spec sheet, rev 10/15/2026)
https://bostondynamics.com/wp-content/uploads/2026/01/atlas-spec-sheet.pdf
- Height 1.9 m · Weight 90 kg · 56 DoF · Reach 2.3 m
- Capacity: 50 kg instant, 30 kg sustained, 20 kg one-handed (for actuator limits later)

## Public (videos/blog)
- Waist, hip yaw and head spin far past human range (2024 reveal video, "Atlas On a Roll" 09/nov/2026) → `waist_yaw`, `hip_yaw`, `neck_yaw` have no limits.
- Wheeled version: wheel = ankle joint that spins continuously ("Atlas On a Roll", https://bostondynamics.com/blog/atlas-on-a-roll/).

## Estimates (v0 placeholders, replace with photo/video measurements)
Segment lengths and masses use human anthropometry (Winter, *Biomechanics and Motor Control of Human Movement*), scaled to H = 1.9 m and m = 90 kg.

| Segment | Length | Mass |
|---|---|---|
| foot (ankle height) | 0.039H = 0.074 m | 1.45% = 1.305 kg |
| shank | 0.246H = 0.467 m | 4.65% = 4.185 kg |
| thigh | 0.245H = 0.466 m | 10% = 9.0 kg |
| hip joint height | 0.530H = 1.007 m | pelvis 14.2% = 12.78 kg |
| shoulder height | 0.818H = 1.554 m | torso (abdomen+thorax) 35.5% = 31.95 kg |
| upper arm | 0.186H = 0.353 m | 2.8% = 2.52 kg |
| forearm | 0.146H = 0.277 m | 1.6% = 1.44 kg |
| hand | 0.108H = 0.205 m | 0.6% = 0.54 kg |
| head | top at 1.9 m | 8.1% = 7.29 kg (robot head likely lighter) |

- Hip joints ±0.10 m, shoulder joints ±0.22 m (widened from 0.21 so the forearm clears the thigh).
- Joint ranges: human-like guesses, except the unlimited ones above.
- Kinematics: 28 body DoF (legs 2×6, arms 2×7, waist yaw, neck yaw). The other 28 of the 56 are assumed to be in the hands, waist and neck; not modeled yet.

## Checks (`atlas/check.py`)
mass 90.00 kg ✓ · height 1.897 m ✓ · overhead reach 2.39 m vs 2.3 m (reach definition unknown) · feet are the only contacts at rest · passive drop stays finite.
