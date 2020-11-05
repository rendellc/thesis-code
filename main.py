#!/usr/bin/env python

import numpy as np

from simulator import VehicleSim
from simulator.common import PID

# vehicle parameters
vp = dict(
        body_mass=1580,
        #front_mass=250,
        #beam_mass=100,
        wheel_mass=200,
        front_length=1,
        front_width=2.830,
        front_height=1,
        beam_length=4.110,
        beam_width=0.3,
        beam_height=0.15,
        wheel_clearing_z=0.1,
        wheel_radius=1.010/2,
        wheel_width=0.4,
        mu1=0.6,
        mu2=0.3,
)



# simulator parameters
sp = dict()

sim = VehicleSim(vp, sp)



# Controller setup
drive_torques = np.zeros(4)
steer_torques = np.zeros(4)

drive_pids = [PID(100,5,5) for _ in range(4)]
steer_pids = [PID(250,50,20) for _ in range(4)]


fps = 50
t, dt, tstop = 0, 1/fps, float('inf')
shouldStop = False
while t < tstop and not shouldStop:
    # Get states
    pos = np.array(sim.getPosition())
    rpy = np.array(sim.getRPY())
    omegas = np.array(sim.getWheelDriveRates())
    steers = np.array(sim.getWheelSteerAngles())
    steerrates = np.array(sim.getWheelSteerRates())

    print(np.round(np.rad2deg(steers), 2))

    # Controllers
    omega_refs = -np.array([1,1,1,1])
    steer_refs = -np.deg2rad(np.array([20,0,0,20]))

    # Wheel Controllers
    for i in range(4):
        drive_torques[i] = drive_pids[i](dt, omega_refs[i] - omegas[i])
        steer_torques[i] = steer_pids[i](dt, steer_refs[i] - steers[i], -steerrates[i])


    # Apply control input
    sim.setTorques(steer_torques, drive_torques)

    shouldStop = sim.step(t, dt)
    t += dt

