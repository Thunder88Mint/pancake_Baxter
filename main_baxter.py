import numpy as np
from setup_baxter import setup_baxter
from flip_pancake_sequence import flip_pancake_sequence
from travel2pancake import travel2pancake
from move_baxter import move_baxter

import kinematics_key_hw07 as kin
import transforms_key_hw04 as tr
np.set_printoptions(precision=4, suppress=True)

# Baxter Libraries
from rad_baxter_limb import RadBaxterLimb
from baxter_interface.limb import Limb
import rospy

if __name__ == "__main__":

    # Inputs
    pancakePosition_in_0 = np.array([1,0,0])
    q = [0,0,0,0,0,0,0]

    # Test IK_full_pose
    K = np.eye(6) * 0.5
    debug = False

    arm = setup_baxter()
    limb = RadBaxterLimb('right')
    limb.set_joint_position_speed(0.8)
    control_rate = rospy.Rate(500)
    
    # Step 0: initial position
    T0 = arm.fk(q)
    p0 = T0[:3,3]
    input('press Enter to see next iteration')

    # Step 1: move to pancake
    travel2pancake(arm)
    T_byPancake = tr.se3(p=[0.8,0,0])
    q1, e, count, successful, msg = arm.ik_full_pose(T_byPancake, q, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
    input('press Enter to see next iteration')
    move_baxter(limb, q1)

    # Step 2: Flip Pancake
    Ts = flip_pancake_sequence(arm, T_byPancake, pancakeLocation=pancakePosition_in_0)
    for i in range(len(Ts)):
        q1, e, count, successful, msg = arm.ik_full_pose(Ts[i], q1, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)
        # viz.add_frame(Ts[i])
        input('press Enter to see next iteration')
        move_baxter(limb, q1)

    

