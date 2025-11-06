## Sudo Code For Pankcake Bot

import numpy as np
import kinematics_key_hw07 as kin
from visualization import VizScene
import transforms_key_hw04 as tr

## Steps that Baxter uses to flip the pancake:
# 1. Move to initial position planar to the pancake
# 2. Follow linear trajectory to slide under the pancake
# 3. Lift pancake vertically
# 4. Rotate wrist to flip pancake
# 5. Retreat to initial position
# 6. slide under pancake again
# 7. Lift pancake vertically again
# 8. Move pancake to plate position
# 9. Flip panckake onto plate

## Baxter DH Parameters
# Parameters Added from Lab1_Part1.py
L = np.array([270.35, 69, 364.35, 69, 374.29, 10, 229.525]) / 1000

dh = [[0, L[0], L[1], -np.pi/2],
       [-np.pi/2, 0, 0, -np.pi/2],
       [0, L[2], -L[3], np.pi/2],
       [0, 0, 0, -np.pi/2],
       [0, L[4], -L[5], np.pi/2],
       [0, 0, 0, -np.pi/2],
       [np.pi, L[6], 0, 0]]

# Orient to position in front of pancake
def go_to_pancake_position():
    pass

# Funciont to slide under pancake
def slide_under_pancake():
    pass

# Function to lift pancake
def lift_pancake():
    pass

# Function to rotate wrist to flip pancake
def rotate_wrist():
    pass    

# Function to retreat to initial position
def initial_position():
    pass

# Function to move pancake to plate position
def move_to_plate_position():
    pass

