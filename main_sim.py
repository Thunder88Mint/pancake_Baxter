import numpy as np
from setup_baxter import setup_baxter
from flip_pancake_sequence import flip_pancake_sequence
# from travel2pancake import travel2pancake
from interpolate_between_positions_in_joint_space import interpolate_between_positions_in_joint_space
from move_baxter_in_visual import move_baxter_in_visual

import kinematics_key_hw07 as kin
import transforms_key_hw04 as tr
np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    visualize = True
    

    # Inputs
    pancakePosition_in_0 = np.array([1,-0.5,0])
    platePosition_in_0 = np.array([1,-1,0])
    q_initial = [np.pi/4,0,0,0,0,0,0]

    # Settings
    K = np.eye(6) * 0.25
    debug = False

    # Setup Robot
    arm = setup_baxter()

    if visualize:
        from visualization import VizScene
        viz = VizScene()
        viz.add_arm(arm)
        viz.add_marker(pancakePosition_in_0)
        grey = np.array([0.3, 0.3, 0.3, 1])
        viz.add_marker(platePosition_in_0,grey)
        viz.add_frame(np.eye(4),'World')
        timeDelay = 10 # seconds
    
    # Step 0: initial position
    T0 = arm.fk(q_initial,base=True)
    p0 = T0[:3,3]

    if visualize:
        viz.update(qs=[q_initial])
        input('press Enter to see next iteration')


    # Step 1: move to pancake
    T_byPancake = tr.se3(p=[0.8,-0.5,0])
    q1, e, count, successful, msg = arm.ik_full_pose(T_byPancake, q_initial, K=K, max_iter=20000, method='pinv', debug=debug, debug_step=debug)
    qs = interpolate_between_positions_in_joint_space(q_initial, q1, 100)
    q_current = qs[-1]

    if visualize:
        move_baxter_in_visual(viz=viz, 
                              qs=qs, 
                              stepHoldTime=0.001, 
                              T_goal=T_byPancake
                            )
        

    # Step 2: Flip Pancake
    Ts = flip_pancake_sequence(arm, T_byPancake, pancakeLocation=pancakePosition_in_0)
    for i in range(len(Ts)):
        q_goal, e, count, successful, msg = arm.ik_full_pose(Ts[i], q_current, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        qs = interpolate_between_positions_in_joint_space(q_current, q_goal, steps=100)
        q_current = qs[-1]

        if visualize:
            move_baxter_in_visual(viz=viz, 
                                  qs=qs, 
                                  stepHoldTime=0.001, 
                                  T_goal=Ts[i]
                                  )
        

