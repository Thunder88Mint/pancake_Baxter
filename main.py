import numpy as np
from setup_baxter import setup_baxter
from flip_pancake_sequence import flip_pancake_sequence
from travel2pancake import travel2pancake

import kinematics_key_hw07 as kin
from visualization import VizScene
import transforms_key_hw04 as tr
np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    pancakePosition_in_0 = np.array([1,0,0])

    q = [0,0,0,0,0,0,0]

    arm = setup_baxter()

    # travel2pancake(arm)
    T_byPancake = tr.se3(p=[0.8,0,0])
    

    Ts = flip_pancake_sequence(arm, T_byPancake, pancakeLocation=pancakePosition_in_0)


    viz = VizScene()
    viz.add_arm(arm)
    viz.update(qs=[q])
    viz.add_marker(pancakePosition_in_0)
    viz.add_frame(np.eye(4),'World')
    viz.add_frame(arm.fk(q,base=True,tip=True))
    viz.add_frame(T_byPancake,'byPancake')
    

    for i in range(len(Ts)):
        viz.add_frame(Ts[i])

    viz.hold()

