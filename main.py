import numpy as np
from setup_baxter import setup_baxter
from flip_pancake_sequence import flip_pancake_sequence

import kinematics_key_hw07 as kin
from visualization import VizScene
import transforms_key_hw04 as tr
np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    arm = setup_baxter()

    q = [0,0,0,0,0,0,0]

    viz = VizScene()
    viz.add_arm(arm)
    viz.update(qs=[q])
    viz.hold()

