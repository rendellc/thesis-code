import numpy as np
from dataclasses import replace

from dynamicmodels import BodyState, WheelState, VehicleState
from simulator import DynSysSim
import utils
import liveplot


bs = BodyState(mass=2500, width=3, length=4, yaw=2*np.pi/4, pos_in=np.array([0,0,0]), vel_b=np.array([0,0,0]))
wbase = WheelState(mass=189, radius=0.8, width=0.4, pos_b=np.array([0,0,0]),
        vel_body_b=bs.vel_b, yawrate_body=bs.yawrate,
        load=utils.weight(bs.mass)/4)
wss = [
        replace(wbase, pos_b=np.array([bs.length/2, bs.width/2,0])),
        replace(wbase, pos_b=np.array([-bs.length/2, bs.width/2,0])),
        replace(wbase, pos_b=np.array([-bs.length/2, -bs.width/2,0])),
        replace(wbase, pos_b=np.array([bs.length/2, -bs.width/2,0])),
]
vs = VehicleState(bs=bs, wss=wss)

animation = liveplot.VehicleAnimation(bs.pos_in, bs.length, bs.width, size=50)

t, dt, tstop = 0, 0.0025, 30
timenextliveplotupdate = t
liveplotfps = 30
while t < tstop:
    steer_torques = [0,0,0,0]
    drive_torques = [0,0,0,0]

    if t < 5:
        drive_torques = np.array([1,1,1,1])*50
        steer_torques = 1*np.array([1,-1,-1,1])
    elif t < 10:
        drive_torques = np.array([-1,-1,0,0])*50
        # steer_torques = 1.3*np.array([-1,1,1,-1])
        steer_torques = 2*np.array([-1,1,1,-1]) # goes wild when wheels turn to 90 deg
    elif t < 15:
        pass
    else:
        steer_torques = [0,0,0,0]
        drive_torques = np.array([1,1,1,1])*50


    # inputs = [drive1,steer1,...,drive4,steer4]
    inputs = utils.interleave([drive_torques, steer_torques])
    vs = DynSysSim.step(vs, inputs, dt)
    t += dt

    slip_ls = [ws.slip_l for ws in vs.wss]
    omegas = [ws.omega for ws in vs.wss]
    wheel_states = [ws.wheel_state for ws in vs.wss]
    # print(wheel_states, slip_ls)

    if t > timenextliveplotupdate:
        # print(wheel_states)
        steer_angles = [ws.steer_angle for ws in vs.wss]

        animation.update(t, vs.bs.pos_in, vs.bs.yaw, steer_angles)
        timenextliveplotupdate = t + 1/liveplotfps

