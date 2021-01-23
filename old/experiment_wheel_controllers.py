#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

#plt.style.use(["science","no-latex"])
plt.style.use(["science","ieee","no-latex"])
figsize = (3.5,2.2)
np.set_printoptions(precision=4)

from simulator import VehicleSim
from simulator.common import PID
from simulator.common import ssa
from simulator.rotations import rot_z, rot_z_der
import liveplot
from blitmanager import BlitManager

import tqdm
import time

import params
vp = params.VEHICLE_PARAMS
sp = params.SIM_PARAMS
sp["do3Dview"] = False
sp["record3Dfilename"] = ""
sp["record3Dfps"] = 30
wheel_radius = vp["wheel_radius"]
wheel_width = vp["wheel_width"]


def run_experiment(mf_factor):
    sim = VehicleSim(vp, sp)

    # Setup to save data
    fps = 30
    tstart, dt, tstop = 0, 1/fps, 300
    #tstart, dt, tstop = 0, 1/fps, 5
    num_steps = int((tstop - tstart)/dt)
    ts = np.zeros(num_steps)
    poss = np.zeros((num_steps,3))
    vels = np.zeros((num_steps,3))
    rpys = np.zeros((num_steps,3))
    rpyrates = np.zeros((num_steps,3))
    omegass = np.zeros((num_steps,4))
    steerss = np.zeros((num_steps,4))
    steerrefs = np.zeros_like(steerss)
    steerratess = np.zeros((num_steps,4))

    # Controller setup
    drive_torques = np.zeros(4)
    steer_torques = np.zeros(4)


    class DriveController:
        def __init__(self, kp, kd, ki):
            self.pid = PID(kp,kd,ki)

        def __call__(self, dt, err, derr=None):
            return self.pid(dt, err, derr)

    drive_ctrls = [DriveController(100,5,5) for _ in range(4)]
    steer_pids = [PID(250,20,10) for _ in range(4)]

    t = tstart
    tPltDraw, pltFps = t, 10
    shouldStop = False
    step_index = 0
    next_print = 0 # debug printing
    #for step_index in range(0,num_steps):
    for step_index in tqdm.tqdm(range(0,num_steps)):
        if shouldStop:
            break
    #while step_index < num_steps and not shouldStop:
        # Get states
        #print(f"{t:.1f}")
        pos = sim.getPosition()
        vel = sim.getVelocity()
        rpy = sim.getRPY()
        rpyrate = sim.getRPYRate()
        omegas = sim.getWheelDriveRates()
        steers = sim.getWheelSteerAngles()
        steerrates = sim.getWheelSteerRates()
        vel_body = rot_z(-rpy[2]) @ vel

        # Save states
        ts[step_index] = t
        poss[step_index] = pos
        vels[step_index] = vel
        rpys[step_index] = rpy
        rpyrates[step_index] = rpyrate
        omegass[step_index] = omegas
        steerss[step_index] = steers
        steerratess[step_index] = steerrates

        # Controllers and References
        vehicle_speed = 0 # km/h
        omega_all = (vehicle_speed/3.6)/wheel_radius
        omega_refs = omega_all*np.array([1,1,1,1])

        steer_angle = 0
        if t > 10:
            steer_angle = 20
        if t > 50:
            steer_angle = -20
        if t > 120:
            steer_angle = -90

        steer_refs = np.deg2rad(steer_angle)*np.array([1,1,1,1])
        steer_error = ssa(steers - steer_refs)

        steerrefs[step_index] = steer_refs

        loads = -sim.getWheelLoads() # negative sign to turn into normal forces
        steer_resistance = 1/3 * np.sign(steer_error)*wheel_width*1*loads

        for i in range(4):
            drive_torques[i] = drive_ctrls[i](dt, omegas[i] - omega_refs[i])

            # TODO: modify steer torque
            steer_torques[i] = mf_factor * steer_resistance[i] + steer_pids[i](dt, steer_error[i], steerrates[i])

        # Apply control input
        sim.setTorques(steer_torques, drive_torques)
        shouldStop = sim.step(t, dt)


        # Setup for next iteration
        t += dt
        step_index += 1


    
    wheel_arms = sim.getWheelPositions()
    datadict = dict(
            t=ts,
            pos=poss,
            vel=vels,
            rpy=rpys,
            rpyrate=rpyrates,
            omegas=omegass,
            steers=steerss,
            steerrates=steerratess,
            steerrefs=steerrefs,
            wheel_arms=wheel_arms,
            order=["FL","RL","RR","FR"],
            mf_factor=mf_factor,
            videofile=sp["record3Dfilename"],
    )

    return datadict


def plot_results(datas, figdir):
    data = datas[0]
    t = data["t"].flatten()

    fig, ax = plt.subplots(1,1,num="Front Left")
    ax.plot(t,np.rad2deg(datas[0]["steerrefs"][:,0]),
            label="ref")
    lines = {}
    for i in range(len(datas)):
        mf_factor = datas[i]['mf_factor'].item()
        line, = ax.plot(t,np.rad2deg(datas[i]["steers"][:,0]),
                label="$K_{mf} = " + str(mf_factor) + "$")
        lines[mf_factor] = line

    ax.set_xlim(t[0],t[~0])
    #ax.set_yticks([90,0,-90,-180])
    ax.set_ylim(-180,45)
    ax.legend(loc="lower left")

    fig.savefig(figdir + "comparison_fl.pdf")

    ax.lines.remove(lines[0])
    ax.legend(loc="lower left")
    ax.set_xlim(170,180)
    ax.set_ylim(-90-5,-90+5)
    fig.savefig(figdir + "comparison_zoom.pdf")

    plt.show()




if __name__=="__main__":
    from scipy.io import savemat, loadmat
    from shutil import move

    mf_list = [0,1,2]

    #for mf in mf_list:
    #    data = run_experiment(mf)
    #    savemat(f"results/wheel_controller/data_{mf}.mat", data)

    datas = [
        loadmat(f"results/wheel_controller/data_{mf}.mat")
        for mf in mf_list
    ]

    plot_results(datas, "results/wheel_controller/")





