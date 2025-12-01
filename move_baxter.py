import numpy as np
import time

# Baxter Libraries
from rad_baxter_limb import RadBaxterLimb
from baxter_interface.limb import Limb
import rospy

def move_baxter(limb, q_des, feedRateHz):
    control_rate = rospy.Rate(feedRateHz)

    start = time.time()
    time_to_wait = 5

    step = 1
    while step < time_to_wait*500:
        limb.set_joint_positions_mod(q_des)
        control_rate.sleep()
        step = step + 1
        



'''
def move_baxter(limb, q_start, q_goal, move_time=3.0):
    rate_hz = 200
    steps = int(move_time * rate_hz)

    # Create smooth joint-space path
    path = []
    for i in range(steps):
        alpha = i / (steps - 1)
        q = (1 - alpha) * np.array(q_start) + alpha * np.array(q_goal)
        path.append(q)

    # Execute
    rate = rospy.Rate(rate_hz)
    for q in path:
        limb.set_joint_positions_mod(q)
        rate.sleep()
'''
