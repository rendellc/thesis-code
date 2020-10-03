import numpy as np
from dynamicmodels import BodyState, WheelState, VehicleState



from dataclasses import replace



bs = BodyState(mass=2500, width=2, length=4, yaw=0, vel_b=np.array([0,0,0]))
wbase = WheelState(mass=189, radius=0.8, width=0.4, pos_b=np.array([0,0,0]), vel_body_b=bs.vel_b, yawrate_body=bs.yawrate)
wss = [
        replace(wbase, pos_b=np.array([bs.length/2, bs.width/2,0])),
        replace(wbase, pos_b=np.array([-bs.length/2, bs.width/2,0])),
        replace(wbase, pos_b=np.array([-bs.length/2, -bs.width/2,0])),
        replace(wbase, pos_b=np.array([bs.length/2, -bs.width/2,0])),
]
vs = VehicleState(bs=bs, wss=wss)


print(vs.states())
print(vs.derivatives([0,0,0,0,0,0,0,0]))

