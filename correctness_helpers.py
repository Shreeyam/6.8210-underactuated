import numpy as np
from quat_helpers import *

def get_residuals(residual_func, x_out, u_out, time_interval, J, Jinv):
    residuals = []
    for i in range(len(x_out) - 1):
        residuals.append(residual_func(x_out[i], x_out[i+1], u_out[i], time_interval, J, Jinv))

    return np.array(residuals)
    
def get_trajectory_sunangle(x_out, c, s):
    q = x_out[:, 3:]

    c_eci = qarray(q, qsandwich, c)

    angle = np.rad2deg(np.arccos(np.dot(c_eci, s)))
    return angle

def get_trajectory_unit_quaternion(x_out):
    q = x_out[:, 3:]
    return np.linalg.norm(q, axis=1)