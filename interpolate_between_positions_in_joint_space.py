import numpy as np

def interpolate_between_positions_in_joint_space(q_initial, q_final, steps):
    
    q_start = np.array(q_initial, dtype=float)
    q_end = np.array(q_final, dtype=float)

    # Create interpolation factors from 0 -> 1
    t = np.linspace(0, 1, steps).reshape(-1, 1)

    # Linear interpolation for each joint dimension
    q_interp = q_start + t * (q_end - q_start)

    return q_interp