
import numpy as np
from numpy import sin, cos, pi

def rotate_x(rad):
    c, s = cos(rad), sin(rad)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]])

def rotate_y(rad):
    c, s = cos(rad), sin(rad)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]])

def rotate_z(rad):
    c, s = cos(rad), sin(rad)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])


def translate(xyz):
    return np.array([
        [1,0,0,xyz[0]],
        [0,1,0,xyz[1]],
        [0,0,1,xyz[2]],
        [0,0,0,1]])

def inv(T):
    Rinv = T[:3,:3].T
    tinv = -Rinv @ T[:3,3]

    Tinv = np.eye(4)
    Tinv[:3,:3] = Rinv
    Tinv[:3,3] = tinv

    return Tinv
#  
#  
#  def rotate_rpy(rpy):
#      Rx = rotate_x(rpy[0])
#      Ry = rotate_y(rpy[1])
#      Rz = rotate_z(rpy[2])
#  
#      return Rz @ Ry @ Rx
#  
#  
#  def transform_xyz(x, y, z):
#      T = np.eye(4)
#      T[:3,3] = np.array([x,y,z])
#      return T
#  
#  def transform_rpy_xyz(roll, pitch, yaw, x, y, z):
#      T = transform_xyz(x,y,z)
#      T[:3,:3] = rotate_rpy(roll, pitch, yaw)
#      return T
#  
#  def transform_xyz_rpy(xyz):
#      T = transform_xyz(*xyz)
#      T[:3,:3] = rotate_rpy(*rpy)
#      return T
#  
#  
#  def transform_cg_to_in(xyz, rpy):
#      R = rotate_rpy(rpy).T
#      t = xyz
#  
#      T = np.eye(4)
#      T[:3,:3] = R
#      T[:3,3] = t
#  
#      return T
#  
#  
#  
#  
#  
#  
#  if __name__ == "__main__":
#      print("testing")
#      print(rotate_rpy(0, 0, 0))
#      print(rotate_rpy(0, 0, pi/2))
#      print(transform_xyz(1, -2, -5))
#      print(transform_rpy_xyz(pi, 0, 0, 1, -2, -5))
#  
#      Lr = 1
#      Lf = 1.5
#      Bf = 2.5
#      h_cg = 2.5
#      delta_wfl = pi/6
#      radius_wheel = 1
#  
#      transform_cg_to_in = np.eye(4)
#      transform_cg_to_in[:3,:3] = rotate_rpy(0,0,0).T
#      transform_cg_to_in[:3,3] = np.array([0,0,0])
#  
#  
#      transform_un_to_cg = np.eye(4)
#      transform_un_to_cg[:3,3] = np.array([-Lr,0,-h_cg])
#  
#      transform_wfl_to_un = np.eye(4)
#      transform_wfl_to_un[:3,:3] = rotate_z(delta_wfl).T
#      transform_wfl_to_un[:3,3] = np.array([Lr+Lf,Bf/2,radius_wheel])
#  
#  
#      transform_wfl_to_in = transform_cg_to_in.dot(transform_un_to_cg).dot(transform_wfl_to_un)
#  
#      print(transform_wfl_to_in)
#  
#  
#  
#  
#  
#  
#  
#  
#      
#      
#  
#  
#  
