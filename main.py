#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from simulator import VehicleSim
from simulator.common import PID
import liveplot

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


# Plot setup
figOmegas, axOmegas = plt.subplots(2,2, num="Angular velocities")
omega_fl = liveplot.Plot([],[], ax=axOmegas[0,0], label=r"$\omega_\text{fl}$")
omega_fl_ref = liveplot.Plot([],[], ax=axOmegas[0,0], label=r"$\omega_\text{fl}^r$", ls="--")
omega_rl = liveplot.Plot([],[], ax=axOmegas[1,0], label=r"$\omega_\text{rl}$")
omega_rl_ref = liveplot.Plot([],[], ax=axOmegas[1,0], label=r"$\omega_\text{rl}^r$", ls="--")
omega_rr = liveplot.Plot([],[], ax=axOmegas[1,1], label=r"$\omega_\text{rr}$")
omega_rr_ref = liveplot.Plot([],[], ax=axOmegas[1,1], label=r"$\omega_\text{rr}^r$", ls="--")
omega_fr = liveplot.Plot([],[], ax=axOmegas[0,1], label=r"$\omega_\text{fr}$")
omega_fr_ref = liveplot.Plot([],[], ax=axOmegas[0,1], label=r"$\omega_\text{fr}^r$", ls="--")

figSteer, axSteer = plt.subplots(2,2, num="Steering angles")
steer_fl = liveplot.Plot([],[], ax=axSteer[0,0], label=r"$\delta_\text{fl}$")
steer_fl_ref = liveplot.Plot([],[], ax=axSteer[0,0], label=r"$\delta_\text{fl}^r$", ls="--")
steer_rl = liveplot.Plot([],[], ax=axSteer[1,0], label=r"$\delta_\text{rl}$")
steer_rl_ref = liveplot.Plot([],[], ax=axSteer[1,0], label=r"$\delta_\text{rl}^r$", ls="--")
steer_rr = liveplot.Plot([],[], ax=axSteer[1,1], label=r"$\delta_\text{rr}$")
steer_rr_ref = liveplot.Plot([],[], ax=axSteer[1,1], label=r"$\delta_\text{rr}^r$", ls="--")
steer_fr = liveplot.Plot([],[], ax=axSteer[0,1], label=r"$\delta_\text{fr}$")
steer_fr_ref = liveplot.Plot([],[], ax=axSteer[0,1], label=r"$\delta_\text{fr}^r$", ls="--")
for ax in axSteer.flatten():
    ax.set_ylim(-np.pi,np.pi)



figTrajectory, axTrajectory = plt.subplots(1,1, num="Trajectory")
trajectoryPlot = liveplot.Plot([],[], ax=axTrajectory, ls="--")
positionMarker = liveplot.Marker(0,0, ax=axTrajectory)
axTrajectory.set_xlim(-20,20)
axTrajectory.set_ylim(-20,20)

plt.show(block=False)

# Controller setup
drive_torques = np.zeros(4)
steer_torques = np.zeros(4)

drive_pids = [PID(100,5,5) for _ in range(4)]
steer_pids = [PID(250,50,20) for _ in range(4)]


fps = 50
t, dt, tstop = 0, 1/fps, float('inf')
tPltDraw, pltFps = t, 10
shouldStop = False
omega_refs = np.array([0,0,0,0])
steer_refs = np.array([0,0,0,0])
while t < tstop and not shouldStop:
    # Get states
    pos = np.array(sim.getPosition())
    rpy = np.array(sim.getRPY())
    omegas = np.array(sim.getWheelDriveRates())
    steers = np.array(sim.getWheelSteerAngles())
    steerrates = np.array(sim.getWheelSteerRates())

    # Plot current state
    omega_fl.update(t,omegas[0])
    omega_fl_ref.update(t,omega_refs[0])
    omega_rl.update(t,omegas[1])
    omega_rl_ref.update(t,omega_refs[1])
    omega_rr.update(t,omegas[2])
    omega_rr_ref.update(t,omega_refs[2])
    omega_fr.update(t,omegas[3])
    omega_fr_ref.update(t,omega_refs[3])

    steer_fl.update(t,steers[0])
    steer_fl_ref.update(t,steer_refs[0])
    steer_rl.update(t,steers[1])
    steer_rl_ref.update(t,steer_refs[1])
    steer_rr.update(t,steers[2])
    steer_rr_ref.update(t,steer_refs[2])
    steer_fr.update(t,steers[3])
    steer_fr_ref.update(t,steer_refs[3])

    trajectoryPlot.update(pos[0],pos[1])
    positionMarker.update(pos[0],pos[1])

    # Controllers
    omega_refs = -2*np.array([1,1,1,1])
    steer_refs = -np.deg2rad(15)*np.array([1,-1,-1,1])

    # Wheel Controllers
    for i in range(4):
        drive_torques[i] = drive_pids[i](dt, omega_refs[i] - omegas[i])
        steer_torques[i] = steer_pids[i](dt, steer_refs[i] - steers[i], -steerrates[i])


    # Apply control input
    sim.setTorques(steer_torques, drive_torques)
    shouldStop = sim.step(t, dt)

    # Update plots
    if t > tPltDraw:
        plt.pause(1e-10)
        tPltDraw = t + 1/pltFps

    # Setup for next iteration
    t += dt

