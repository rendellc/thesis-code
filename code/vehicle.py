import numpy as np

import transform as tf


class Wheel:
    def __init__(self, position, vehicle, config):
        self.position = position 
        self.vehicle = vehicle
        self.angle = 0*np.pi/180
        self.wheel_radius = config["wheel_radius"]
        self.wheel_width = config["wheel_width"]


        self.wheel_to_cg = tf.translate(self.position) @ tf.rotate_z(self.angle)



    def __str__(self):
        return f"Wheel({self.position}, {self.angle})"
    def __repr__(self):
        return self.__str__()


class Car2D:
    def __init__(self, config):
        self.mass = config["mass"]
        self.width_front = config["width_front"]
        self.width_rear = config["width_rear"]
        self.length_front = config["length_front"]
        self.length_rear = config["length_rear"]

        self.position = np.array([0,0,0])
        self.velocity = np.array([0,0,0])
        # note rpy[0] = rpy[1] = 0 always in this model
        self.rpy = np.array([0,0,0])
        self.rpy_rate = np.array([0,0,0])


        lf, lr = self.length_front, self.length_rear
        wf, wr = self.width_front, self.width_rear
        self.wheels = [
                #Wheel(np.array([lf, -wf/2, 0]), self, config),
                Wheel(np.array([lf, wf/2, 0]), self, config),
                #Wheel(np.array([-lr, -wr/2, 0]), self, config),
                #Wheel(np.array([lr, wr/2, 0]), self, config)
                ]


        cg_to_in = tf.translate(self.position) @ tf.rotate_z(-self.rpy[2])

        for wheel in self.wheels:
            wheel_to_in = wheel.wheel_to_cg @ cg_to_in

            print((wheel_to_in @ np.array([0,0,0,1]))[:3])
            print((wheel_to_in @ np.array([1,0,0,1]))[:3])


