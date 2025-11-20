import numpy as np
import time

# Baxter Libraries
from rad_baxter_limb import RadBaxterLimb
from baxter_interface.limb import Limb
import rospy

def move_baxter(limb, q_des):

    start = time.time()
    time_to_wait = 5

    step = 1
    while step < time_to_wait*500:
        limb.set_joint_positions_mod(q_des)
        step = step + 1
