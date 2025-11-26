import numpy as np
from setup_baxter import setup_baxter
from flip_pancake_sequence import flip_pancake_sequence
from travel2pancake import travel2pancake

import kinematics_key_hw07 as kin
from visualization import VizScene
import transforms_key_hw04 as tr
np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    # Inputs
    pancakePosition_in_0 = np.array([1,0,0])
    platePosition_in_0 = np.array([4,4,4])
    q_initial = [0,0,0,0,0,0,0]

    K = np.eye(6) * 0.5
    debug = False

    # Setup
    arm = setup_baxter()

    viz = VizScene()
    viz.add_arm(arm)
    viz.add_marker(pancakePosition_in_0)
    viz.add_frame(np.eye(4),'World')
    timeDelay = 10 # seconds
    
    # Step 0: initial position
    T0 = arm.fk(q_initial)
    p0 = T0[:3,3]
    viz.update(qs=[q_initial])
    input('press Enter to see next iteration')

    # Step 1: move to pancake
    travel2pancake(arm)
    T_byPancake = tr.se3(p=[0.8,0,0])
    q1, e, count, successful, msg = arm.ik_full_pose(T_byPancake, q_initial, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
    viz.update(qs=[q1])
    input('press Enter to see next iteration')

    # Step 2: Flip Pancake
    Ts = flip_pancake_sequence(arm, T_byPancake, pancakeLocation=pancakePosition_in_0)
    for i in range(len(Ts)):
        q1, e, count, successful, msg = arm.ik_full_pose(Ts[i], q1, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        viz.add_frame(Ts[i])
        viz.update(qs=[q1])
        input('press Enter to see next iteration')

    
    '''
    Ts_new = pathPlanning_interpolation(Ts[0], Ts[1])   # Adjust elevation
    for i in range(len(Ts_new)):
        q1, e, count, successful, msg = arm.ik_full_pose(Ts[i], q1, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        viz.update(qs=[q1])
        input('press Enter to see next iteration')

    trajectoryPlanning(Ts[1], Ts[2], v)   # Scoop Pancake
    for i in range(len(Ts_new)):
        q1, e, count, successful, msg = arm.ik_full_pose(Ts[i], q1, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        viz.update(qs=[q1])
        input('press Enter to see next iteration')

    pathPlanning_interpolation(Ts[2], Ts[3])    # Lift Pancake
    for i in range(len(Ts_new)):
        q1, e, count, successful, msg = arm.ik_full_pose(Ts[i], q1, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        viz.update(qs=[q1])
        input('press Enter to see next iteration')

    pathPlanning(Ts[3], Ts[4])    # Flip   (Probably needs trajectory planning)
    for i in range(len(Ts_new)):
        q1, e, count, successful, msg = arm.ik_full_pose(Ts[i], q1, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        viz.update(qs=[q1])
        input('press Enter to see next iteration')
    '''
