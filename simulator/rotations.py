import math
import math

import numpy as np

c = math.cos
s = math.sin

def rot_x(phi):
    return np.array([
        [1, 0, 0],
        [0, c(phi), -s(phi)],
        [0, s(phi), c(phi)]])
def rot_y(theta):
    return np.array([
        [c(theta), 0, s(theta)],
        [0, 1, 0],
        [-s(theta), 0, c(theta)]])
def rot_z(psi):
    return np.array([
        [c(psi), -s(psi), 0],
        [s(psi), c(psi), 0],
        [0, 0, 1]])

def euler_to_matrix(rpy):
    return rot_z(rpy[2]) @  rot_y(rpy[1]) @ rot_x(rpy[0])


