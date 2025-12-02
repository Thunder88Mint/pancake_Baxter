import numpy as np
import transforms_key_hw04 as tr
import kinematics_key_hw07 as kin

def setup_baxter():
    L = np.array([270.35, 69, 364.35, 69, 374.29, 10, 229.525]) / 1000

    # dh = [[0, L[0], L[1], -np.pi/2],
    #     [-np.pi/2, 0, 0, -np.pi/2],
    #     [0, L[2], -L[3], np.pi/2],
    #     [0, 0, 0, -np.pi/2],
    #     [0, L[4], -L[5], np.pi/2],
    #     [0, 0, 0, -np.pi/2],
    #     [np.pi, L[6], 0, 0]]
    
    dh = [[0, L[0], L[1], -np.pi/2],
        [np.pi/2, 0, 0, np.pi/2],
        [0, L[2], L[3], -np.pi/2],
        [0, 0, 0, np.pi/2],
        [0, L[4], L[5], -np.pi/2],
        [0, 0, 0, np.pi/2],
        [0, L[6], 0, 0]]


    T0_in_torso = [[0.7071, 0.7071, 0, 0.06353],
                [-0.7071, 0.7071, 0, -0.2597],
                [0, 0, 1, 0.119],
                [0, 0, 0, 1]]
    
    

    # Spatula Dimensions
    L1 = 0.25 # m
    L2 = 0.1 # m

    # Spatula Tip Transform
    spatulaAngle = 10 # degrees
    R = tr.roty(-np.pi/2)
    T1 = tr.se3(R)
    T2 = tr.se3(tr.roty(-spatulaAngle*np.pi/180), p=[L1,0,0])
    T3 = tr.se3(p=[L2,0,0])
    T_tip = T1 @ T2 @ T3

    # add baxter joint limits
        # Rows = joints [S0, S1, E0, E1, W0, W1, W2]
        # Cols = [min, max] in radians
    joint_limits = np.array([
        [-2.4609,  0.8901],   # S0
        [-2.1468,  1.0472],   # S1
        [-3.0194,  3.0194],   # E0
        [-0.0524,  2.6180],   # E1
        [-3.0543,  3.0543],   # W0
        [-1.5708,  2.0944],   # W1
        [-3.0543,  3.0543]    # W2
    ])  

    arm = kin.SerialArm(dh, tip=T_tip, base=T0_in_torso, joint_limits=joint_limits)


    return arm