#!/usr/bin/env python

from simulator import VehicleSim
from simulator.common import PID



# vehicle parameters
vp = dict(
        front_mass=250,
        beam_mass=100,
        wheel_mass=50,
        front_length=1,
        front_width=2.440,
        front_height=1,
        beam_length=4.110,
        beam_width=0.3,
        beam_height=0.15,
        wheel_clearing_z=0.1,
        wheel_radius=0.4,
        wheel_width=0.2,
        mu1=1,
        mu2=0.6,
)

# simulator parameters
sp = dict()

sim = VehicleSim(vp, sp)

fps = 50
t, dt, tstop = 0, 1/fps, float('inf')
shouldStop = False
while t < tstop and not shouldStop:

    steer_torques = [0,0,0,0]
    drive_torques = [0,0,0,0]

    sim.setTorques(steer_torques, drive_torques)

    shouldStop = sim.step(t, dt)
    t += dt

