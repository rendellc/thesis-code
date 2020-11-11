#!/usr/bin/env python

import numpy as np

from simulator import VehicleSim
from simulator.common import PID

"""
Lengde: 4110 mm
Lengde mellom akslinger 2830 mm.
Hjuldiameter er 1010 mm.
Det gjør at fronten er 270 mm foran framhjulene eller 775 mm foran framaksel.

Bredde er på 2400 mm i ytterkant av hjulene og det er 2000 mm mellom senter av hjulene så da tolker jeg at hjulene er 400 mm brede.

Vekt er 2380 kg uten nyttelast. Opp mot 2000 kg ekstra med nyttelast. Maks vekt altså ~4400 kg
"""
axle_to_axle = 2.830
wheel_diameter = 1.010
total_length = 4.110
front_axle_to_front = 0.775
wheel_width = 0.400
total_width = 2.400

wheel_radius = wheel_diameter/2

# vehicle parameters
vp = dict(
        body_mass=1580,
        wheel_mass=200,
        front_length=1,
        front_width=total_width - wheel_width - wheel_radius,
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
sp = dict(
        body_color=[0.7,0.7,0.7],
	do3Dview=True,
)

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

    #print(np.round(np.rad2deg(steers), 2))

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

