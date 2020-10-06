from models.body import BodyModel
from models.wheel import WheelModel
from models.vehicle import VehicleModel

import numpy as np
import utils

def createAutoagriModel(pos_in, yaw, vel_b, yawrate):
    bs = BodyModel(mass=2500, width=3, length=4, yaw=yaw, pos_in=pos_in, vel_b=vel_b)
    wbase = WheelModel(mass=189, radius=0.8, width=0.4, pos_b=np.array([0,0,0]),
            vel_body_b=bs.vel_b, yawrate_body=bs.yawrate,
            load=utils.weight(bs.mass)/4)
    wss = [
            wbase.from_changes(pos_b=np.array([bs.length/2, bs.width/2,0])),
            wbase.from_changes(pos_b=np.array([-bs.length/2, bs.width/2,0])),
            wbase.from_changes(pos_b=np.array([-bs.length/2, -bs.width/2,0])),
            wbase.from_changes(pos_b=np.array([bs.length/2, -bs.width/2,0])),
    ]

    return VehicleModel(bs, wss)
