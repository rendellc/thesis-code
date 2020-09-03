import numpy as np
import transform as tf
from wheel import *

g = 9.81

class Car2D:
    def __init__(self, config):
        self.mass = config["mass"]
        self.width_front = config["width_front"]
        self.width_rear = config["width_rear"]
        self.length_front = config["length_front"]
        self.length_rear = config["length_rear"]

        # Compute moment of inertia matrix
        # pretend its a prism rotating about CG for now
        M = self.mass
        W = (self.width_front + self.width_rear)/2
        L = self.length_front + self.length_rear
        H = config["height"]
        Ixx = (1/12)*M*(H**2 + W**2)
        Iyy = (1/12)*M*(H**2 + L**2)
        Izz = (1/12)*M*(L**2 + W**2)
        self.inertia = np.array([
            [Ixx, 0, 0],
            [0, Iyy, 0],
            [0, 0, Izz]])

        self.position = config["cog_start_position"]  # in inertial frame
        self.velocity = config["cog_start_velocity"] # in inertial frame
        # note rpy[0] = rpy[1] = 0 always in this model
        self.rpy = config["cog_start_rpy"]
        self.rpy_rate = config["cog_start_rpy_rate"]

        lf, lr = self.length_front, self.length_rear
        wf, wr = self.width_front, self.width_rear
        self.wheels = {
                "FL": Wheel(np.array([lf, wf/2, 0]), self, config),
                "FR": Wheel(np.array([lf, -wf/2, 0]), self, config),
                "RL": Wheel(np.array([-lr, wr/2, 0]), self, config),
                "RR": Wheel(np.array([-lf, -wr/2, 0]), self, config),
                }

        self.reset()



    def reset(self):
        self.net_force_cg = np.array([0.,0.,0.])
        self.net_torque = np.array([0.,0.,0.])

        self.cg_to_in = tf.translate(self.position) @ tf.rotate_z(-self.rpy[2])
        self.cg_to_in_deriv = tf.translate_deriv(self.velocity) @ tf.rotate_z(-self.rpy[2]) \
                + tf.translate(self.position) @ tf.rotate_z_deriv(-self.rpy[2], -self.rpy_rate[2])


        
    def add_wheel_force_and_torque(self):
        for name, wheel in self.wheels.items():
            wheel_to_in = wheel.wheel_to_cg @ self.cg_to_in
            wheel.compute_friction_force(self.mass*g/len(self.wheels))
            wheel_torque = np.cross(wheel.position, wheel.force_cg)

            self.net_force_cg += wheel.force_cg
            self.net_torque += wheel_torque 



    def compute_derivatives(self):
        self.net_force_cg = np.array([0.,0.,0.])
        self.net_torque = np.array([0.,0.,0.])

        self.add_wheel_force_and_torque()
        self.accel = self.cg_to_in[:3,:3] @ self.net_force_cg / self.mass
        self.rpy_accel = np.linalg.inv(self.inertia) @ self.net_torque

    def step_forward(self, stepsize):
        self.position += stepsize*self.velocity
        self.velocity += stepsize*self.accel
        self.rpy[2] += stepsize*self.rpy_rate[2]
        self.rpy_rate[2] += stepsize*self.rpy_accel[2]

    def get_simulatable_children(self):
        return list(self.wheels.values())


    def __repr__(self):
        return f"Car({self.position}, {self.velocity}, {self.rpy})"
