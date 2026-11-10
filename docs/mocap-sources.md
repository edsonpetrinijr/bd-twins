# Mocap sources

- **CMU Graphics Lab Motion Capture Database** (mocap.cs.cmu.edu), BVH conversion by B. Hahne (cgspeed),
  mirrored at github.com/una-dinosauria/cmu-mocap. License text (READMEFIRST.txt): "This data is free for
  use in research and commercial projects worldwide." Acknowledgment: "The data used in this project was
  obtained from mocap.cs.cmu.edu. The database was created with funding from NSF EIA-0196217."
- Clips in `mocap/raw/`: `02_01` walk; `69_70` walk up to object, squat, pick up, set down elsewhere;
  `69_72` walk up, pick up, set down.
- Not used: LAFAN1 (CC BY-NC-ND 4.0: retargeting is a derivative work), AMASS (non-commercial license).

Retargeting: `mocap/retarget.py` (BVH FK, leg-length scale, per-frame IK on atlas.xml). Output CSV + mjlab
tracking NPZ.
