import numpy as np
import transforms_key_hw04 as tr
from kinematics_key_hw07 import SerialArm

def flip_pancake_sequence(arm: SerialArm, T_cur=np.eye(4), pancakeLocation=np.zeros(3)):
    """Full sequence of steps Baxter uses to flip a pancake."""

    # T_cur = arm.fk(q,base=True, tip=True)
    p_cur = T_cur[:3,3]

    ### STEP 1: Scoop pancake under spatula
    distance2Pancake = pancakeLocation - p_cur
    # Align Virtically to pancake
    m1 = [0, 0, distance2Pancake[2]]
    T1 = tr.se3(p=m1)
    T1_in_0 = T_cur @ T1
    m2 = [distance2Pancake[0], distance2Pancake[1], 0]
    T2 = tr.se3(p=m2)
    T2_in_0 = T1_in_0 @ T2

    # STEP 2: Lift pancake virtically
    m3 = [0, 0, 0.2]
    T3 = tr.se3(p=m3)
    T3_in_0 = T2_in_0 @ T3
    
    # STEP 3: Flip pancake by rotating wrist
    xRotation = tr.rotx(-np.pi/2)
    T4 = tr.se3(xRotation)
    T4_in_0 = T3_in_0 @ T4

    Ts = [T1_in_0, T2_in_0, T3_in_0, T4_in_0]
    return Ts



