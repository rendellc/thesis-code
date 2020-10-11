from models.body import BodyModel
from models.wheel import WheelModel
from models.vehicle import VehicleModel

import numpy as np
import utils

def createAutoagriModel(pos_in, yaw, vel_b, yawrate, wheel_angles = []):
    body = BodyModel(mass=2500, width=3, length=4, yaw=yaw, pos_in=pos_in, vel_b=vel_b)
    wbase = WheelModel(mass=189, radius=0.8, width=0.4, pos_b=np.array([0,0,0]),
            vel_body_b=body.vel_b, yawrate_body=body.yawrate,
            load=utils.weight(body.mass)/4)

    steer_angles = [0,0,0,0]
    for i in range(len(wheel_angles)):
        steer_angles[i] = wheel_angles[i]

    wheels = [
            wbase.from_changes(pos_b=np.array([body.length/2, body.width/2,0]), steer_angle=steer_angles[0]),
            wbase.from_changes(pos_b=np.array([-body.length/2, body.width/2,0]), steer_angle=steer_angles[1]),
            wbase.from_changes(pos_b=np.array([-body.length/2, -body.width/2,0]), steer_angle=steer_angles[2]),
            wbase.from_changes(pos_b=np.array([body.length/2, -body.width/2,0]), steer_angle=steer_angles[3]),
    ]

    return VehicleModel(body, wheels)
