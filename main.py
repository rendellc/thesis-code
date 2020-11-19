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
        wheel_mass_inner=100,
        wheel_mass_tire=100,
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

doLivePlots = True
if doLivePlots:
    # Plot setup
    figOmegas, axOmegas = plt.subplots(2,2, num="Angular velocity")
    omega_fl, = axOmegas[0,0].plot([],[], animated=True, label=r"$\omega_\text{fl}$")
    omega_fl_ref, = axOmegas[0,0].plot([],[], animated=True, label=r"$\omega_\text{fl}^r$", ls="--")
    omega_rl, = axOmegas[1,0].plot([],[], animated=True, label=r"$\omega_\text{rl}$")
    omega_rl_ref, = axOmegas[1,0].plot([],[], animated=True, label=r"$\omega_\text{rl}^r$", ls="--")
    omega_rr, = axOmegas[1,1].plot([],[], animated=True, label=r"$\omega_\text{rr}$")
    omega_rr_ref, = axOmegas[1,1].plot([],[], animated=True, label=r"$\omega_\text{rr}^r$", ls="--")
    omega_fr, = axOmegas[0,1].plot([],[], animated=True, label=r"$\omega_\text{fr}$")
    omega_fr_ref, = axOmegas[0,1].plot([],[], animated=True, label=r"$\omega_\text{fr}^r$", ls="--")
    omega_axes = [axOmegas[0,0],axOmegas[1,0],axOmegas[1,1],axOmegas[0,1]]
    omega_lines = [omega_fl, omega_rl, omega_rr, omega_fr]
    omega_ref_lines = [omega_fl_ref, omega_rl_ref, omega_rr_ref, omega_fr_ref]

    figSteers, axSteers = plt.subplots(2,2, num="Steering angle")
    steer_fl, = axOmegas[0,0].plot([],[], animated=True, label=r"$\delta_\text{fl}$")
    steer_fl_ref, = axOmegas[0,0].plot([],[], animated=True, label=r"$\delta_\text{fl}^r$", ls="--")
    steer_rl, = axOmegas[1,0].plot([],[], animated=True, label=r"$\delta_\text{rl}$")
    steer_rl_ref, = axOmegas[1,0].plot([],[], animated=True, label=r"$\delta_\text{rl}^r$", ls="--")
    steer_rr, = axOmegas[1,1].plot([],[], animated=True, label=r"$\delta_\text{rr}$")
    steer_rr_ref, = axOmegas[1,1].plot([],[], animated=True, label=r"$\delta_\text{rr}^r$", ls="--")
    steer_fr, = axOmegas[0,1].plot([],[], animated=True, label=r"$\delta_\text{fr}$")
    steer_fr_ref, = axOmegas[0,1].plot([],[], animated=True, label=r"$\delta_\text{fr}^r$", ls="--")
    steer_axes = [axSteers[0,0],axSteers[1,0],axSteers[1,1],axSteers[0,1]]
    steer_lines = [steer_fl, steer_rl, steer_rr, steer_fr]
    steer_ref_lines = [steer_fl_ref, steer_rl_ref, steer_rr_ref, steer_fr_ref]

    # Draw empty plots
    plt.show(block=False)
    plt.pause(0.1)

    # Copy backgrounds for blitting
    figOmegasBg = figOmegas.canvas.copy_from_bbox(figOmegas.bbox)
    axOmegaBgs = [
            figOmegas.canvas.copy_from_bbox(ax.bbox)
            for ax in axOmegas.flatten()
    ]
    axSteerBgs = [
            figSteers.canvas.copy_from_bbox(ax.bbox)
            for ax in axSteers.flatten()
    ]

# Controller setup
drive_torques = np.zeros(4)
steer_torques = np.zeros(4)

drive_pids = [PID(100,5,5) for _ in range(4)]
steer_pids = [PID(250,20,10) for _ in range(4)]


fps = 50
t, dt, tstop = 0, 1/fps, float('inf')
tPltDraw, pltFps = t, 10
shouldStop = False
while t < tstop and not shouldStop:
    # Get states
    pos = np.array(sim.getPosition())
    rpy = np.array(sim.getRPY())
    omegas = np.array(sim.getWheelDriveRates())
    steers = np.array(sim.getWheelSteerAngles())
    steerrates = np.array(sim.getWheelSteerRates())

    # Controllers and references
    omega_refs = -1*np.array([1,1,1,1])
    steer_refs = -np.deg2rad(5)*np.array([1,-1,-1,1])

    # Update live plots
    if doLivePlots:
        ## 1) restore backgrounds
        for bg in axOmegaBgs:
            figOmegas.canvas.restore_region(bg)
        for bg in axSteerBgs:
            figSteers.canvas.restore_region(bg)

        ## 2) update plot data
        for i, (omega_line, ref_line) in enumerate(zip(omega_lines, omega_ref_lines)):
            omega_line.set_data(
                    np.append(omega_line.get_xdata(), t),
                    np.append(omega_line.get_ydata(), omegas[i])
            )
            ref_line.set_data(
                    np.append(omega_fl_ref.get_xdata(), t),
                    np.append(omega_fl_ref.get_ydata(), omega_refs[i])
            )

        for i, (steer_line, ref_line) in enumerate(zip(steer_lines, steer_ref_lines)):
            steer_line.set_data(
                    np.append(steer_line.get_xdata(), t),
                    np.append(steer_line.get_ydata(), steers[i])
            )
            ref_line.set_data(
                    np.append(steer_fl_ref.get_xdata(), t),
                    np.append(steer_fl_ref.get_ydata(), steer_refs[i])
            )

        ## 3) set axes limits
        for i, ax in enumerate(omega_axes):
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            ax.set_xlim(min(xlim[0], t), max(xlim[1], t))
            ax.set_ylim(min(ylim[0], omegas[i], omega_refs[i]), max(ylim[1], omegas[i], omega_refs[i]))
        for i, ax in enumerate(steer_axes):
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            ax.set_xlim(min(xlim[0], t), max(xlim[1], t))
            ax.set_ylim(min(ylim[0], steers[i], steer_refs[i]), max(ylim[1], steers[i], steer_refs[i]))

        ## 4) draw plots
        for i in range(len(omega_axes)):
            omega_axes[i].draw_artist(omega_lines[i])
            omega_axes[i].draw_artist(omega_ref_lines[i])
        for i in range(len(steer_axes)):
            steer_axes[i].draw_artist(steer_lines[i])
            steer_axes[i].draw_artist(steer_ref_lines[i])

        # TODO: check for memory leak
        figOmegas.canvas.blit(figOmegas.bbox) 
        figSteers.canvas.blit(figSteers.bbox) 

        figOmegas.canvas.flush_events()
        figSteers.canvas.flush_events()

    # Wheel Controllers
    for i in range(4):
        drive_torques[i] = drive_pids[i](dt, omega_refs[i] - omegas[i])
        steer_torques[i] = steer_pids[i](dt, steer_refs[i] - steers[i], -steerrates[i])


    # Apply control input
    sim.setTorques(steer_torques, drive_torques)
    shouldStop = sim.step(t, dt)

    # Setup for next iteration
    t += dt

