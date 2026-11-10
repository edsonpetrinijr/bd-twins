# Atlas v0: where each number comes from

Rule: every number in `atlas/atlas.xml` is either from a public source or marked as an estimate.

## Public (Boston Dynamics spec sheet, rev 10/15/2026)
https://bostondynamics.com/wp-content/uploads/2026/01/atlas-spec-sheet.pdf
- Height 1.9 m · Weight 90 kg · 56 DoF · Reach 2.3 m
- Capacity: 50 kg instant, 30 kg sustained, 20 kg one-handed (for actuator limits later)

## Public (videos/blog)
- Waist, hip yaw and head spin far past human range (2024 reveal video, "Atlas On a Roll" 09/nov/2026) → `waist_yaw`, `hip_yaw`, `neck_yaw` have no limits.
- Wheeled version: wheel = ankle joint that spins continuously ("Atlas On a Roll", https://bostondynamics.com/blog/atlas-on-a-roll/).

## Measured on spec-sheet photos (v0.1, 09/nov)
Method: pixel coordinates of joint centers on the spec-sheet renders, scale from a known length. Images extracted with `pdfimages` (not committed).
- **Side view** (page 2, robot at shelf; sole y=683 px, head top y=120 px → 1.9 m = 563 px, 296 px/m). Legs almost straight (~8-10° bend), lengths corrected for the bend.
  - ankle joint (blue disc) 625 px → **ankle height 0.196 m** (human ratio gave 0.074: biggest change)
  - knee 527 px, hip pitch actuator 390 px → **shin 0.336 m**, **thigh 0.467 m**, **hip height 0.999 m**
  - shoulder ~215 px → **shoulder height 1.58 m**
  - foot ~95 px long, ankle 37 px from heel → **foot 0.32 m**, ankle ~0.04 m behind foot center
- **Walking render** (page 1, 433 px/m from 823 px = 1.9 m): hip 0.97 m (matches side view), forearm ~0.28 m, hand ~0.15 m, upper arm ~0.275 m (foreshortened).
- **Reach 2.3 m** taken as overhead reach → upper arm + forearm + hand = 2.3 − 1.58 = 0.72 m → **0.28 / 0.28 / 0.16 m**. So the reach check now passes by construction, not as an independent test.
- **Widths (low confidence)**: front render (page 2) is seen from above, head looks bigger. Head ≈ 0.22 m wide, torso ≈ 0.28 m, shoulder joints ±0.20 m, hip joints ±0.09 m (also set so forearm clears thigh).

## Still estimates
Masses use human anthropometry (Winter, *Biomechanics and Motor Control of Human Movement*), scaled to H = 1.9 m and m = 90 kg.

Lengths in this table are the old v0 human values; v0.1 uses the measured ones above.

| Segment | Length (v0, superseded) | Mass |
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

- Joint ranges: human-like guesses, except the unlimited ones above.
- Kinematics: 28 body DoF (legs 2×6, arms 2×7, waist yaw, neck yaw). The other 28 of the 56 are assumed to be in the hands, waist and neck; not modeled yet.

## Checks (`atlas/check.py`)
mass 90.00 kg ✓ · height 1.899 m ✓ · overhead reach 2.30 m (by construction) · feet are the only contacts at rest · passive drop stays finite.

## Visual shell (v0.2, 09/nov)
Only looks; no effect on physics. Visual geoms are massless, group 2, no collision. Collision geoms (group 3) are unchanged and carry all mass.
- Shapes and colors read off the spec-sheet renders (pages 1-2): blue shell + dark grey frame, cylinder head with front light ring and antenna, back battery pack with blue light strips, carry handle over the back, ribbed actuators at hip/elbow/lower thigh, blue discs on the outer side of knee and ankle, long dark shin slab, 3 fingers + thumb.
- Rounded/tapered parts are convex-hull meshes generated in `atlas/build.py` (`shell()`), no mesh files.
- Part sizes are eyeballed from the renders, side by side with `atlas/render.py`. Not measured to the mm.

## Silhouette fit (v0.3, 09/nov)
The spec-sheet PDF carries an exact alpha mask for the walking render on page 1 (`pdfimages` → `p-004.png`).
`atlas/compare.py` fits camera + pose to that mask (proportions are not fitted) and reports IoU, total and per band.
Overlay: white = both, red = only real Atlas (model missing volume), cyan = only model (model extra).

| Round | Change | IoU total | head | torso+arms | legs | feet |
|---|---|---|---|---|---|---|
| 0 | v0.2 | 0.796 | 0.718 | 0.845 | 0.782 | 0.729 |
| 1 | shorter head, thinner backpack, chest/belly forward, longer sole, wider shin | 0.808 | 0.745 | 0.866 | 0.775 | 0.754 |
| 2 | **shoulder joint 0.521 → 0.45 above waist (world 1.51 m), upper arm 0.28 → 0.31** (sweep, best of 12) | 0.835 | 0.775 | 0.883 | 0.830 | 0.724 |
| 3 | head forward, handle as U over the back, bulkier upper arm/elbow, shin slab forward | 0.833 | 0.880 | 0.865 | 0.820 | 0.717 |
| 4 | shorter handle, bigger elbow, taller blue belly, blue battery modules on back (seen on page 1 back view), thicker neck | 0.837 | 0.882 | 0.871 | 0.821 | 0.723 |

Notes:
- Shoulder height from the side photo (1.58 m) was biased: arm raised there. Silhouette fit says ~1.51 m.
- With shoulder 1.51 m and arm 0.31+0.28+0.16 m, overhead fingertip reach = 2.26 m vs spec 2.3 m. Spec does not say how reach is measured.
- One view only. Widths (head, torso, shoulders, hips) are still checked from a single 3/4 view.

## More references (v0.4, 09/nov)
Downloaded, not committed (BD copyright). Used only to read shapes and positions.
- bostondynamics.com images: `atlas-hero-2026-scaled.jpg` (real robot, front), `atlas-render-blog.jpg` (side, hi-res of spec-sheet page 2), `Webinar-March-Atlas-form_and_funtion-Promo_Assets-0226_HP-scaled.png` (3/4 front, standing, best leg detail), `atlas-battery-walk-real-collage-768x1245-1.png` (back 3/4 + front), `atlas-wheels-rl-split-screen.jpg` (BD's own sim render).
- Frames from "Atlas | Product Features" (YouTube rrUHZKlrxms), e.g. t=25 s (reach line 0→2.3 m = overhead fingertip with arm up), t=73 s (head + shoulders, front).
- `atlas/views.py` puts each reference next to the model from a similar camera.

What they changed:
- **Shoulder**: big dark actuator on the top corner of the torso, arm hangs straight below it. Front close-up (head ring Ø ≈ 0.20 m as scale): center 0.227 m below head center → **1.573 m**, lateral ≈ 0.25 m. Used **0.23 m** (0.25 dropped IoU on the 3/4 silhouette; the two disagree by ~2 cm).
- **Reach 2.3 m** = fingertip with the arm straight up (video frame). Model: 2.32 m.
- **Torso** wider (±0.175 m), yoke as a broad dark dome, blue belly, ribbed band between, dark ribbed waist below.
- **Pelvis** = horizontal dark drums (axis y) with ribbed centers; thighs hang from their ends.
- **Legs**: slim blue thigh with a ribbed dark motor at the back; **knee and ankle = blue drums on the outside** with a dark center; **shin = dark rounded paddle on the inside**, in front of the knee-ankle line; short dark foot.
- **Hip** spacing 0.13 m (front photo suggests up to 0.16; kept 0.13 so the hanging forearm clears the thigh).
- Hip yaw/roll moved 0.08 m above the hip pitch axis (front discs sit above the thigh drum). Pelvis-thigh contact excluded (they are no longer parent/child).

IoU after these: 0.82 (was 0.84 at v0.3). Lower on purpose: v0.3 fit one 3/4 silhouette by pushing the shoulder too low; v0.4 follows five views.

## Mass (v0.5, 09/nov)
No public breakdown. Per-body estimates, sum = spec-sheet 90 kg. Inside each body, mass is spread over its visual shells (inertia and COM follow the shapes), battery packs 2.5x denser than the rest of the torso. Frozen as explicit `<inertial>` in the XML.

| Body | kg | Why |
|---|---|---|
| torso | 36 | 2 battery packs (4 h runtime), compute, waist actuator |
| pelvis | 8 | hip actuators + frame |
| head | 4 | cameras, lights, neck |
| hip (x2) | 0.6 | link between hip roll and pitch |
| thigh (x2) | 8.5 | knee actuator lives in the thigh (heavy near the hip, like most legged robots) |
| shin (x2) | 4 | ankle actuators |
| foot (x2) | 1.5 | |
| upper arm (x2) | 3.2 | elbow + shoulder yaw actuators |
| forearm (x2) | 2.0 | wrist actuators |
| hand (x2) | 1.2 | 3 fingers + thumb with motors |

First try (uniform density over the shells) gave a 7 kg shin and a 4 kg forearm with a 4 kg thigh: shapes alone put mass far from the hips, which no real legged robot does. Replaced by the table above.

## Actuators (v0.6, 09/nov) — estimates
`atlas/atlas_mjlab.py`. No public torque numbers. Sized from the spec-sheet loads plus margin for the dynamic moves in BD videos (jumps, flips):
- Knee/hip pitch: deep squat with 50 kg "instant" load, 140 kg total, ~0.25 m lever, two legs → ~170 Nm; x2.5 for jumps → **400 Nm**.
- Shoulder pitch/roll: 20 kg one-handed at ~0.7 m + arm weight → ~160 Nm → **200 Nm**.
- Others scaled down: hip roll 300, ankle pitch 250, waist 200, hip yaw 150, elbow 120, ankle roll 120, shoulder yaw 100, wrist 30, neck 10 Nm.
- Reflected inertia 0.002–0.1 kg m²; PD gains from it (10 Hz, damping ratio 2), the mjlab convention.

## mjlab (v0.6)
- `atlas/atlas_mjlab.py` registers `Mjlab-Velocity-Flat-BD-Atlas` and `Mjlab-Velocity-Rough-BD-Atlas` on mjlab's velocity-tracking task.
- Smoke test on Mac CPU: env builds, 28 actions, 200 steps with zero action, no NaN, no fall. Pelvis sags 5–10 cm with zero action (PD at 10 Hz is soft under 90 kg); a trained policy compensates.
- mjlab pinned to the BD fork commit in `requirements.txt`.

## v0.8 (09/nov): narrower shoulders
Edson: shoulders looked too wide. The front close-up widths (shoulder 0.25, torso ±0.175) were all measured in head-ring units with ring Ø = 0.20 m assumed; scaled torso widths by ~0.85 instead: shoulder 0.205 m, belly ±0.148, yoke ±0.152, hips 0.11 m. Collision radii trimmed so the hanging forearm still clears the thigh and the upper arm clears the torso.

## CAD parts (v0.9, 09/nov)
`atlas/cad.py` (build123d) makes real solids for the visual shell and writes `atlas/meshes/*.stl`; `build.py` references them. First parts: shin slab (stadium profile, filleted edges) and knee/ankle drums (filleted, recessed hub). Same shapes and sizes as before, now with soft edges and real holes. Geometry is ours, drawn from the BD photos; the Sketchfab model is only looked at like another photo, never traced or measured into a part.

## v1.2 (09/nov)
Thicker legs (thigh 0.15 x 0.13 m, shin slab 0.066 m thick, bigger knee/ankle/hip drums; hips 0.12 m apart), domed yoke (3-section loft), swappable battery modules and a swept carry handle in CAD, wider neck, upper-arm shell starts below the shoulder drum, muted colors closer to the product renders, studio lights in `atlas/stage.py`. IoU 0.82 (head 0.87).

## v1.3 (09/nov): leg fidelity pass
From the leg close-ups (webinar 3/4 render + spec-sheet side render):
- Hip roll actuator is a ribbed drum along x at the top of the thigh (round end seen from the front, ribs from the side). The old y-axis hip drum was removed.
- Thigh: blue full-depth shell on the upper ~60%; lower part = blue strip in front + dark finned motor block behind (vertical fins seen from the side). Narrower laterally (0.10 m), deep front-back (0.14 m).
- Shin slab bigger front-back (r 0.088/0.074 m) and further forward (0.06 m ahead of the knee-ankle line), thinner laterally (0.054 m).
IoU 0.82 (legs 0.77, feet 0.75).

## v1.5 (09/nov): legs no longer stick out
Edson: legs looked "pushed out". Two causes: knee/ankle drums sat 3–4 cm outboard of the thigh (side "ears" from the front), and the shin slab sat 6 cm ahead of the knee-ankle line (shin jutting forward from the side). Drums pulled inboard so their outer face is flush with the thigh; slab moved to 0.025 m ahead and slightly smaller. Legs IoU 0.77 → 0.82, total 0.82.
