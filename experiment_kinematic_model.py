#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from simulator import VehicleSim
from simulator.common import PID
from simulator.common import ssa
from simulator.rotations import rot_z, rot_z_der
import liveplot
from blitmanager import BlitManager

import time

np.set_printoptions(precision=4)

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
        wheel_mass_inner=100,
        wheel_mass_tire=100,
        front_length=1,
        front_width=total_width - wheel_width - wheel_radius - 0.3,
        front_height=1,
        front_track=total_width - wheel_width, # distance between front wheel centers
        rear_track=total_width - wheel_width, # distance between rear wheel centers
        beam_length=4.110,
        beam_width=0.2,
        beam_height=0.3,
        wheel_clearing_z=0.1,
        wheel_radius=1.010/2,
        wheel_width=0.4,
        mu1=0.8,
        mu2=0.3,
)


# simulator parameters
sp = dict(
        body_color=[0.7,0.7,0.7],
        do3Dview=False,
        substeps=2,
        window_width=400,
        window_height=400,
        friction_scale=0.01,
)
sim = VehicleSim(vp, sp)

wheel_arms = sim.getWheelPositions()

# Kinematic models
pos = sim.getPosition()
vel = sim.getVelocity()
_,_,yaw = sim.getRPY()

# Full model
A = np.zeros((8,3))
zu = np.array([0,0,1])
A[::2,0] = 1
A[1::2,1] = 1
A[:,2] = np.cross(zu,wheel_arms)[:,:2].flatten()
Ainv = np.linalg.inv(A.T.dot(A)).dot(A.T)

full_state = np.hstack([pos[:2],yaw]).flatten()
full_J = np.eye(3) # transformation from body to inertial frame
full_J[:2,:2] = rot_z(yaw)[:2,:2]

# Simple model
Ax = A[:,0]
Ay = A[:,1]
Ap = A[:,2]
Axinv = Ax.T/(Ax.T.dot(Ax))
Ayinv = Ay.T/(Ay.T.dot(Ay))
Apinv = Ap.T/(Ap.T.dot(Ap))

simple_state = full_state.copy()
simple_J = full_J.copy()
simple_bodyrate = np.zeros(3)

b = np.zeros(8)

# Controller setup
drive_torques = np.zeros(4)
steer_torques = np.zeros(4)

drive_pids = [PID(100,5,5) for _ in range(4)]
steer_pids = [PID(250,20,10) for _ in range(4)]

fps = 50
t, dt, tstop = 0, 1/fps, 60
tPltDraw, pltFps = t, 10
shouldStop = False
while t < tstop and not shouldStop:
    # Get states
    pos = sim.getPosition()
    vel = sim.getVelocity()
    rpy = sim.getRPY()
    rpyrate = sim.getRPYRate()
    omegas = sim.getWheelDriveRates()
    steers = sim.getWheelSteerAngles()
    steerrates = sim.getWheelSteerRates()
    vel_body = rot_z(-rpy[2]) @ vel

    # Compute longitudinal slip, eq 8.9 in Kiencke
    # assume roll and pitch are zero
    wheel_velocities_body = vel_body + wheel_arms @ rot_z_der(-rpy[2], -rpyrate[2])
    wheel_betas = np.arctan2(wheel_velocities_body[:,1], wheel_velocities_body[:,0])
    wheel_alphas = steers - wheel_betas
    wheel_speeds = np.linalg.norm(wheel_velocities_body, axis=1)
    wheel_rot_speeds = omegas * wheel_radius * np.cos(wheel_alphas)
    wheel_speed_diffs = wheel_rot_speeds - wheel_speeds
    wheel_max_speeds = np.maximum(np.abs(wheel_rot_speeds), np.abs(wheel_speeds))
    # dont generate divide by zero warnings
    with np.errstate(divide="ignore",invalid="ignore"):
        wheel_slips_lo = wheel_speed_diffs/wheel_max_speeds
    # nans come from zeros, leave infs
    wheel_slips_lo[np.isnan(wheel_slips_lo)] = 0 


    # Kinematic model setup
    b[::2] = wheel_radius*omegas*np.cos(steers)
    b[1::2] = wheel_radius*omegas*np.sin(steers)

    # Full kinematic model
    ## compare with GT
    full_error_pos = full_state[:2] - pos[:2]
    full_error_yaw = abs(ssa(full_state[2] - rpy[2]))

    ## update for next timestep
    full_bodyrate = Ainv.dot(b) #[vx,vy,yawrate]
    full_J[:2,:2] = rot_z(full_state[2])[:2,:2]
    full_state = full_state + \
            dt*full_J.dot(full_bodyrate)

    # Simplified kinematic model
    ## compare with GT
    simple_error_pos = simple_state[:2] - pos[:2]
    simple_error_yaw = abs(ssa(simple_state[2] - rpy[2]))

    ## update for next timestep
    simple_bodyrate[0] = Axinv.dot(b)
    simple_bodyrate[1] = Ayinv.dot(b)
    simple_bodyrate[2] = Apinv.dot(b)
    simple_J[:2,:2] = rot_z(simple_state[2])[:2,:2]
    simple_state = simple_state + \
            dt*simple_J.dot(simple_bodyrate)

    
    se, fe = np.linalg.norm(simple_error_pos), np.linalg.norm(full_error_pos)
    ratio = se/fe
    #print(f"{se:.5f} {fe:.5f} {ratio}")

    # Controllers and References
    omega_all = 0
    if t > 5:
        omega_all = 1

    omega_refs = omega_all*np.array([1,1,1,1])
    #steer_refs = np.deg2rad(10)*np.sin(0.1*t)*np.array([1,-1,-1,1])
    steer_refs = np.deg2rad(90)*np.array([1,-1,-1,1])



    for i in range(4):
        drive_torques[i] = drive_pids[i](dt, omegas[i] - omega_refs[i])
        steer_torques[i] = steer_pids[i](dt, ssa(steers[i] - steer_refs[i]), steerrates[i])


    # Apply control input
    sim.setTorques(steer_torques, drive_torques)
    shouldStop = sim.step(t, dt)

    # Setup for next iteration
    t += dt



