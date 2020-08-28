
import numpy as np
from numpy import sin, cos, pi

def rotate_x(rad):
    return np.array([
        [1, 0, 0],
        [0, cos(rad), -sin(rad)],
        [0, sin(rad), cos(rad)]])

def rotate_y(rad):
    return np.array([
        [cos(rad), 0, sin(rad)],
        [0, 1, 0],
        [-sin(rad), 0, cos(rad)]])

def rotate_z(rad):
    return np.array([
        [cos(rad), -sin(rad), 0],
        [sin(rad), cos(rad), 0],
        [0, 0, 1]])

def rotate_rpy(roll, pitch, yaw):
    Rx = rotate_x(roll)
    Ry = rotate_y(pitch)
    Rz = rotate_z(yaw)

    return Rz.dot(Ry.dot(Rx))


def transform_xyz(x, y, z):
    T = np.eye(4)
    T[:3,3] = np.array([x,y,z])
    return T

def transform_rpy_xyz(roll, pitch, yaw, x, y, z):
    T = transform_xyz(x,y,z)
    T[:3,:3] = rotate_rpy(roll, pitch, yaw)
    return T



if __name__ == "__main__":
    print("testing")
    print(rotate_rpy(0, 0, 0))
    print(rotate_rpy(0, 0, pi/2))
    print(transform_xyz(1, -2, -5))
    print(transform_rpy_xyz(pi, 0, 0, 1, -2, -5))

    Lr = 1
    Lf = 1.5
    Bf = 2.5
    h_cg = 2.5
    delta_wfl = pi/6
    radius_wheel = 1

    transform_cg_to_in = np.eye(4)
    transform_cg_to_in[:3,:3] = rotate_rpy(0,0,0).T
    transform_cg_to_in[:3,3] = np.array([0,0,0])


    transform_un_to_cg = np.eye(4)
    transform_un_to_cg[:3,3] = np.array([-Lr,0,-h_cg])

    transform_wfl_to_un = np.eye(4)
    transform_wfl_to_un[:3,:3] = rotate_z(delta_wfl).T
    transform_wfl_to_un[:3,3] = np.array([Lr+Lf,Bf/2,radius_wheel])


    transform_wfl_to_in = transform_cg_to_in.dot(transform_un_to_cg).dot(transform_wfl_to_un)

    print(transform_wfl_to_in)








    
    



