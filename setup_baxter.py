import numpy as np
import kinematics_key_hw07 as kin

def setup_baxter():
    L = np.array([270.35, 69, 364.35, 69, 374.29, 10, 229.525]) / 1000

    dh = [[0, L[0], L[1], -np.pi/2],
        [-np.pi/2, 0, 0, -np.pi/2],
        [0, L[2], -L[3], np.pi/2],
        [0, 0, 0, -np.pi/2],
        [0, L[4], -L[5], np.pi/2],
        [0, 0, 0, -np.pi/2],
        [np.pi, L[6], 0, 0]]


    T0_in_torso = [[0.7071, 0.7071, 0, 0.06353],
                [-0.7071, 0.7071, 0, -0.2597],
                [0, 0, 1, 0.119],
                [0, 0, 0, 1]]

    arm = kin.SerialArm(dh, base=T0_in_torso)

    return arm