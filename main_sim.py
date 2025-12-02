import numpy as np
from setup_baxter import setup_baxter
from flip_pancake_sequence import flip_pancake_sequence
from travel2pancake import travel2pancake
from compute_path2pancake import compute_path2pancake

import kinematics_key_hw07 as kin
import transforms_key_hw04 as tr
np.set_printoptions(precision=4, suppress=True)


grey = np.array([0.3, 0.3, 0.3, 1])


if __name__ == "__main__":

    # Inputs
    pancakePosition_in_w = np.array([1,-0.5,0])
    platePosition_in_w = np.array([1,-1,0])
    q_initial = [0,0,0,0,0,0,0]
    tablePosition_in_w = pancakePosition_in_w - np.array([0,0,1])
    tableInfluenceRadius = 1

    # Settings
    visualize = True
    K = np.eye(6) * 0.25
    debug = False

    # Setup
    arm = setup_baxter()

    if visualize:
        from visualization import VizScene
        viz = VizScene()
        viz.add_arm(arm)
        viz.add_frame(arm.get_base_transform(),'Base')
        viz.add_marker(pancakePosition_in_w)
        viz.add_marker(platePosition_in_w,grey)
        viz.add_frame(np.eye(4),'World')
        # viz.add_obstacle(tablePosition_in_w, rad=tableInfluenceRadius)
        # timeDelay = 10 # seconds
    

    # Step 0: initial position
    q_current = q_initial
    T0 = arm.fk(q_initial,base=True)
    p0 = T0[:3,3]
    if visualize:
        viz.update(qs=[q_initial])
        # input('press Enter to see next iteration')


    # Step 1: move to pancake
    # GPT
    # 1. Target TIP pose in world
    T_tip_target_world = tr.se3(p=pancakePosition_in_w)

    # 2. Convert world -> base
    T_tip_target_base = tr.inv(arm.get_base_transform()) @ T_tip_target_world

    # 3. Remove the tool-tip transform
    T_linkN_target_base = T_tip_target_base @ tr.inv(arm.get_tip_transform())

    # 4. Extract the xyz goal
    goal = T_linkN_target_base[:3, 3]
    # goal = T_tip_target_base[:3,3]

    # print("Final goal in BASE frame:", goal)


    obst_location_in_0 = (tr.inv(arm.get_base_transform()) @ np.append(tablePosition_in_w,1))[:3]


    # print("Base transform:\n", arm.get_base_transform())
    # print("Tip transform:\n", arm.get_tip_transform())
    # print("T_tip_target_world:\n", T_tip_target_world)
    # print("T_tip_target_base:\n", T_tip_target_base)
    # print("T_linkN_target_base:\n", T_linkN_target_base)
    # print("GOAL (base frame):", goal)


    qs = compute_path2pancake(
        arm=arm, 
        q_init=q_current, 
        goal=goal,
        obst_location=obst_location_in_0,
        obst_radius=tableInfluenceRadius
        )


    if visualize:
        print('steps: ', len(qs))
        for i in range(len(qs)):
            # print('Progress: ', i+1, ' of ', len(qs))
            viz.update(qs=[qs[i]])
            viz.hold(0.00625)
        input('press Enter to see next iteration')


    # Step 1.1: Angle Spatula
    q_current = qs[-1]
    q_new, _, _, _, _ = arm.ik_full_pose(T_tip_target_world, q_current, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
    if visualize:
        viz.update(qs=[q_new])
        input('press Enter to see next iteration')




'''

    # travel2pancake(arm)
    
    # q1, e, count, successful, msg = arm.ik_full_pose(T_byPancake, q_initial, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
    # if visualize:
    #     viz.update(qs=[q1])
    #     input('press Enter to see next iteration')


    # q2 = [1.5777, -0.2128, -0.2976, 1.7737, -2.6465, 0.9607, 0.8007]
    T_check = arm.fk(q1, base=True, tip=True)
    print('Should be: ', [0.8,0,0])
    print('Is: ', T_check[:3,3])
    # input('press Enter to see next iteration')

    # Step 2: Flip Pancake
    Ts = flip_pancake_sequence(arm, T_byPancake, pancakeLocation=pancakePosition_in_w)
    for i in range(len(Ts)):
        q1, e, count, successful, msg = arm.ik_full_pose(Ts[i], q1, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        print(q1)
        viz.add_frame(Ts[i])
        viz.update(qs=[q1])
        input('press Enter to see next iteration')

    

'''










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

