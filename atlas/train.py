"""mjlab train/play with the Atlas tasks registered. Usage: python atlas/train.py [train|play] TASK [mjlab args]"""

import sys

import atlas_carry  # noqa: F401  (registers the carry tasks)
import atlas_mimic  # noqa: F401  (registers the mocap tracking task)
import atlas_mjlab  # noqa: F401  (registers the tasks)

mode = sys.argv.pop(1) if len(sys.argv) > 1 and sys.argv[1] in ("train", "play") else "train"
if mode == "play":
    from mjlab.scripts.play import main
else:
    from mjlab.scripts.train import main
main()
