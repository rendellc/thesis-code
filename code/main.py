import numpy as np
import matplotlib.pyplot as plt

from models.autoagri import createAutoagriModel

import simulator
import utils
import liveplots


model = createAutoagriModel(
        pos_in=[0,0,0],
        yaw=0,
        vel_b=[0,0,0],
        yawrate=0)

fig, axs = plt.subplots(1,2)
animationXY = liveplots.VehicleAnimation(model.bs.pos_in, model.bs.length, model.bs.width, size=20, ax=axs[0])
timeseriesOmega0 = liveplots.TimeSeries([],[], label=r"$\omega_0$", ax=axs[1])


plt.show(block=False)

solver: simulator.Solver = simulator.ImprovedEuler()

t, dt, tstop = 0, 0.005, 30
timenextliveupdate = t
liveplotfps = 30
while t < tstop:
    omegas = np.array([ws.omega for ws in model.wss])

    steer_torques = [0,0,0,0]
    drive_torques = [0,0,0,0]

    if t < 5:
        drive_torques = np.array([1,1,1,1])*100
        steer_torques = 2*np.array([1,-1,-1,1])
    elif t < 10:
        drive_torques = np.array([-1,-1,0,0])*100
        # steer_torques = 1.3*np.array([-1,1,1,-1])
        steer_torques = 4*np.array([-1,1,1,-1]) # goes wild when wheels turn to 90 deg
    elif t < 15:
        pass
    else:
        steer_torques = [0,0,0,0]
        #drive_torques = np.array([1,1,1,1])*50


    # inputs = [drive1,steer1,...,drive4,steer4]
    inputs = utils.interleave([drive_torques, steer_torques])
    model = solver.step(model, inputs, dt)
    t += dt

    slip_ls = [ws.slip_l for ws in model.wss]
    omegas = [ws.omega for ws in model.wss]
    wheel_states = [ws.wheel_state for ws in model.wss]
    # print(wheel_states, slip_ls)

    if t > timenextliveupdate:
        # print(wheel_states)
        steer_angles = [ws.steer_angle for ws in model.wss]

        animationXY.update(t, model.bs.pos_in, model.bs.yaw, steer_angles)
        timeseriesOmega0.update(t, omegas[0])

        plt.pause(dt) # TODO: blit for better FPS

        timenextliveupdate = t + 1/liveplotfps


