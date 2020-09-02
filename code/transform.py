
import numpy as np
from numpy import sin, cos, pi

def rotate_x(rad):
    c, s = cos(rad), sin(rad)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]])

def rotate_x_deriv(rad, rad_rate):
    c, s = cos(rad), sin(rad)
    return rad_rate*np.array([
        [0, 0, 0, 0],
        [0, -s, -c, 0],
        [0, c, -s, 0],
        [0, 0, 0, 0]])


def rotate_y(rad):
    c, s = cos(rad), sin(rad)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]])

def rotate_y_deriv(rad, rad_rate):
    c, s = cos(rad), sin(rad)
    return rad_rate*np.array([
        [-s, 0, c, 0],
        [0, 0, 0, 0],
        [-c, 0, -s, 0],
        [0, 0, 0, 0]])

def rotate_z(rad):
    c, s = cos(rad), sin(rad)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

def rotate_z_deriv(rad, rad_rate):
    c, s = cos(rad), sin(rad)
    return rad_rate*np.array([
        [-s, -c, 0, 0],
        [c, -s, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]])


def translate(xyz):
    return np.array([
        [1,0,0,xyz[0]],
        [0,1,0,xyz[1]],
        [0,0,1,xyz[2]],
        [0,0,0,1]])

def translate_deriv(xyz_rate):
    return np.array([
        [0,0,0,xyz_rate[0]],
        [0,0,0,xyz_rate[1]],
        [0,0,0,xyz_rate[2]],
        [0,0,0,0]])

def inv(T):
    Rinv = T[:3,:3].T
    tinv = -Rinv @ T[:3,3]

    Tinv = np.eye(4)
    Tinv[:3,:3] = Rinv
    Tinv[:3,3] = tinv

    return Tinv


def to_homogeneous(vec):
    return np.hstack([vec, 1])



