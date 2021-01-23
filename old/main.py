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
        do3Dview=True,
        substeps=2,
        window_width=400,
        window_height=400,
        friction_scale=1,
)

sim = VehicleSim(vp, sp)

wheel_arms = sim.getWheelPositions()

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
    bmOmegas = BlitManager(figOmegas.canvas, [*omega_lines, *omega_ref_lines])

    figSteers, axSteers = plt.subplots(2,2, num="Steering angle")
    steer_fl, = axSteers[0,0].plot([],[], animated=True, label=r"$\delta_\text{fl}$")
    steer_fl_ref, = axSteers[0,0].plot([],[], animated=True, label=r"$\delta_\text{fl}^r$", ls="--")
    steer_rl, = axSteers[1,0].plot([],[], animated=True, label=r"$\delta_\text{rl}$")
    steer_rl_ref, = axSteers[1,0].plot([],[], animated=True, label=r"$\delta_\text{rl}^r$", ls="--")
    steer_rr, = axSteers[1,1].plot([],[], animated=True, label=r"$\delta_\text{rr}$")
    steer_rr_ref, = axSteers[1,1].plot([],[], animated=True, label=r"$\delta_\text{rr}^r$", ls="--")
    steer_fr, = axSteers[0,1].plot([],[], animated=True, label=r"$\delta_\text{fr}$")
    steer_fr_ref, = axSteers[0,1].plot([],[], animated=True, label=r"$\delta_\text{fr}^r$", ls="--")
    steer_axes = [axSteers[0,0],axSteers[1,0],axSteers[1,1],axSteers[0,1]]
    steer_lines = [steer_fl, steer_rl, steer_rr, steer_fr]
    steer_ref_lines = [steer_fl_ref, steer_rl_ref, steer_rr_ref, steer_fr_ref]
    bmSteers = BlitManager(figSteers.canvas, [*steer_lines, *steer_ref_lines])
    for ax in steer_axes:
        ax.set_ylim(-np.pi, np.pi)

    figSlips, axSlips = plt.subplots(2,2, num="Slip x")
    slip_fl, = axSlips[0,0].plot([],[], animated=True, label=r"$s_\text{fl}$")
    slip_rl, = axSlips[1,0].plot([],[], animated=True, label=r"$s_\text{rl}$")
    slip_rr, = axSlips[1,1].plot([],[], animated=True, label=r"$s_\text{rr}$")
    slip_fr, = axSlips[0,1].plot([],[], animated=True, label=r"$s_\text{fr}$")
    slip_axes = [axSlips[0,0],axSlips[1,0],axSlips[1,1],axSlips[0,1]]
    slip_lines = [slip_fl, slip_rl, slip_rr, slip_fr]
    bmSlips = BlitManager(figSlips.canvas, slip_lines)
    for ax in slip_axes:
        ax.set_ylim(-1,1)


    figFricScale, axFricScale = plt.subplots(1,1, num="Friction scale")
    fric_scale_line, = axFricScale.plot([],[], animated=True)
    bmFricScale = BlitManager(figFricScale.canvas, [fric_scale_line])
    axFricScale.set_ylim(0,1.5)

    # Draw empty plots
    plt.show(block=False)
    plt.pause(0.1)

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



    # Controllers and References
    omega_all = 2
    if t > 10:
        omega_all = -2

    # Change sim parameters
    fs = 1
    if 10 <= t < 20:
        fs = 0.5

    omega_refs = omega_all*np.array([1,1,1,1])
    steer_refs = np.deg2rad(0)*np.sin(0.1*t)*np.array([1,-1,-1,1])


    for i in range(4):
        drive_torques[i] = drive_pids[i](dt, omegas[i] - omega_refs[i])
        steer_torques[i] = steer_pids[i](dt, ssa(steers[i] - steer_refs[i]), steerrates[i])


    # Update live plots
    if doLivePlots:
        ## update plot data
        for i, (omega_line, ref_line) in enumerate(zip(omega_lines, omega_ref_lines)):
            omega_line.set_data(
                    np.append(omega_line.get_xdata(), t),
                    np.append(omega_line.get_ydata(), omegas[i])
            )
            ref_line.set_data(
                    np.append(ref_line.get_xdata(), t),
                    np.append(ref_line.get_ydata(), omega_refs[i])
            )
        for i, (steer_line, ref_line) in enumerate(zip(steer_lines, steer_ref_lines)):
            steer_line.set_data(
                    np.append(steer_line.get_xdata(), t),
                    np.append(steer_line.get_ydata(), steers[i])
            )
            ref_line.set_data(
                    np.append(ref_line.get_xdata(), t),
                    np.append(ref_line.get_ydata(), steer_refs[i])
            )
        for i, slip_line in enumerate(slip_lines):
            slip_line.set_data(
                    np.append(slip_line.get_xdata(), t),
                    np.append(slip_line.get_ydata(), wheel_slips_lo[i])
            )

        fric_scale_line.set_data(
            np.append(fric_scale_line.get_xdata(), t),
            np.append(fric_scale_line.get_ydata(), fs)
        )


        ## set axes limits
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
        for ax in slip_axes:
            xlim = ax.get_xlim()
            ax.set_xlim(min(xlim[0], t), max(xlim[1], t))

        xlim = axFricScale.get_xlim()
        axFricScale.set_xlim(min(xlim[0], t), max(xlim[1], t))


        ## update plots
        bmOmegas.update()
        bmSteers.update()
        bmSlips.update()
        bmFricScale.update()

    # Apply control input
    sim.setFrictionScale(fs)
    sim.setTorques(steer_torques, drive_torques)
    shouldStop = sim.step(t, dt)

    # Setup for next iteration
    t += dt

