import numpy as np
from setup_baxter import setup_baxter
from flip_pancake_sequence import flip_pancake_sequence
from travel2pancake import travel2pancake

import kinematics_key_hw07 as kin
from visualization import VizScene
import transforms_key_hw04 as tr
np.set_printoptions(precision=4, suppress=True)

# Baxter Libraries
from rad_baxter_limb import RadBaxterLimb
from baxter_interface.limb import Limb
import rospy

if __name__ == "__main__":

    # Setup Baxter
    limb = RadBaxterLimb('right')
    limb.set_joint_position_speed(0.8)
    control_rate = rospy.Rate(500)


    pancakePosition_in_0 = np.array([1,0,0])

    q = [0,0,0,0,0,0,0]

    arm = setup_baxter()

<<<<<<< HEAD
    # Eli's Functions
    # travel2pancake(arm)
    T_byPancake = tr.se3(p=[0.8,0,0])
    
    # Cody's Function
    Ts = flip_pancake_sequence(arm, T_byPancake, pancakeLocation=pancakePosition_in_0)
=======

    q_test = [np.pi/4]*7
    T_test = arm.fk(q_test, base=True, tip=True)

    viz = VizScene()
    viz.add_arm(arm)
    viz.update(qs=[q_test])
    viz.add_frame(np.eye(4),'World')
    viz.add_marker(T_test[:3,3])
    
    # T0 = arm.fk(q)
    # p0 = T0[:3,3] - np.array([-0.4, 0, 0])

    # travel2pancake(arm)
    # T_byPancake = tr.se3(p=[0.8,0,0])

    
    # Test IK_full_pose
    K = np.eye(6) * 0.5
    debug = True
    # q1, e, count, successful, msg = arm.ik_full_pose(T_test, q, K=K, max_iter=10000, method='J_T', debug=debug, debug_step=debug)
    q1, e, count, successful, msg = arm.ik_full_pose(T_test, q, K=K, max_iter=10000, method='pinv', debug=debug, debug_step=debug)





    # Ts = flip_pancake_sequence(arm, T_byPancake, pancakeLocation=pancakePosition_in_0)
>>>>>>> baxterCommand3


    viz = VizScene()
    viz.add_arm(arm)
    viz.update(qs=[q1])
    viz.add_marker(pancakePosition_in_0)
    viz.add_frame(np.eye(4),'World')
    viz.add_frame(arm.fk(q,base=True,tip=True))
    viz.add_frame(T_byPancake,'byPancake')
    

    # for i in range(len(Ts)):
    #     viz.add_frame(Ts[i])

    viz.hold()

