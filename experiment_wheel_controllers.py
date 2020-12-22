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
wheel_mass = vp["wheel_mass_inner"] + vp["wheel_mass_tire"]


def run_experiment(mf_factor,steer_poles,steer_reference):
    sim = VehicleSim(vp, sp)

    # Setup to save data
    fps = 30
    tstart, dt, tstop = 0, 1/fps, 150
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
    steer_torques = np.zeros((num_steps,4))
    inertia_steer = 0.25*wheel_mass*(wheel_radius**2 + 1/3*wheel_width**2)

    #steer_poles = inertia_steer*steer_poles_no_inertia # desired poles of system, can be complex
    #print("Inertia steer:", inertia_steer)
    #print("Steer PID:", steer_poles[1],steer_poles[2],steer_poles[3])
    kd = steer_poles[2]
    kp = steer_poles[1]
    ki = steer_poles[0]
    print("Steer PID:", kp,ki,kd)

    class DriveController:
        def __init__(self, kp, kd, ki):
            self.pid = PID(kp,ki,kd)

        def __call__(self, dt, err, derr=None):
            return self.pid(dt, err, derr)

    drive_ctrls = [DriveController(100,5,5) for _ in range(4)]
    steer_pids = [PID(kp,ki,kd) for _ in range(4)]

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

        steer_angle = steer_reference(t)
        steer_refs = np.deg2rad(steer_angle)*np.array([1,1,1,1])
        steer_error = ssa(steers - steer_refs)

        steerrefs[step_index] = steer_refs

        loads = -sim.getWheelLoads() # negative sign to turn into normal forces
        sign = np.sign
        #sign = lambda x: np.pi/2*np.arctan(x/0.001)
        steer_resistance = 1/3 * sign(steer_error)*wheel_width*1*loads

        for i in range(4):
            drive_torques[i] = drive_ctrls[i](dt, omegas[i] - omega_refs[i])

            # TODO: modify steer torque
            steer_torques[step_index,i] = mf_factor * steer_resistance[i] + steer_pids[i](dt, steer_error[i], steerrates[i])

        # Apply control input
        sim.setTorques(steer_torques[step_index], drive_torques)
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
            steertorques=steer_torques,
            wheel_arms=wheel_arms,
            order=["FL","RL","RR","FR"],
            mf_factor=mf_factor,
            videofile=sp["record3Dfilename"],
    )

    return datadict


def plot_results(datas, figdir):
    data = datas[0]
    t = data["t"].flatten()

    fig, ax = plt.subplots(1,1,num="Front Left",clear=True)
    ax.plot(t,np.rad2deg(datas[0]["steerrefs"][:,0]), label=r"$\delta_w^*$")
    lines = {}
    for i in range(len(datas)):
        mf_factor = datas[i]['mf_factor'].item()
        line, = ax.plot(t,np.rad2deg(datas[i]["steers"][:,0]),
                label="$K_{mf} = " + str(mf_factor) + "$")
        lines[mf_factor] = line

    ax.set_xlim(t[0],t[~0])
    #ax.set_yticks([90,0,-90,-180])
    ax.set_ylim(-60,45)
    ax.legend(loc="lower left")

    fig.savefig(figdir + "comparison_fl.pdf")

    zoom_tlim = (250,251)
    final_deg = -40
    zoom_deglim = (final_deg - 1,final_deg + 1)

    #ax.lines.remove(lines[0])
    ax.legend(loc="lower left")
    ax.set_xlim(*zoom_tlim)
    ax.set_ylim(*zoom_deglim)
    fig.savefig(figdir + "comparison_zoom.pdf")


    fig, ax = plt.subplots(1,1,num="Front Left tau_s",clear=True)
    ax.plot([],[]) # empty plot to get the same colors as above
    for i in range(len(datas)):
        mf_factor = datas[i]['mf_factor'].item()
        ax.plot(t,datas[i]["steertorques"][:,0],
                label="$K_{mf} = " + str(mf_factor) + "$")

        #ax.set_xlim(t[0],t[~0])
    ax.set_xlim(*zoom_tlim)
    ax.ticklabel_format(style="sci")
    #ax.set_yticks([90,0,-90,-180])
    #ax.set_ylim(-50000,50000)
    ax.legend(loc="upper left")
    fig.savefig(figdir + "steer_torque_fl.pdf")


    #plt.show()




if __name__=="__main__":
    from scipy.io import savemat, loadmat
    from shutil import move

    def step(ts, y0, y1):
        def f(t):
            if t < ts:
                return y0
            
            return y1

        return f

    def zero(t): 
        return 0

    reference = step(10,0,5)

    inertia_steer = 0.25*wheel_mass*(wheel_radius**2 + 1/3*wheel_width**2)
    mf = 0.05
    #steer_poles = inertia_steer*np.poly1d([-2,-2+0.1j,-2-0.1j],True)
    # [_,kd,kp,ki]
    steer_poles = np.poly1d([1,200,4000,20])

    if False:
        # Use only MF
        print("Only Mf")
        mf_list = [mf,1]
        for mf in mf_list:
            data = run_experiment(mf,np.poly1d([0,0,0],True), reference)
            savemat(f"results/wheel_controller/data_only_mf_{mf}.mat", data)

        # Combined
        print("Combined")
        mf_list = [0,mf]
        for mf in mf_list:
            data = run_experiment(mf,steer_poles, reference)
            savemat(f"results/wheel_controller/data_combined_{mf}.mat", data)

    #mf_list = [0.5,0.75,1]
    #mf_list = [0.75,1,1.25]
    mf_list = [0,mf]
    datas = [
        loadmat(f"results/wheel_controller/data_combined_{mf}.mat")
        for mf in mf_list
    ]

    plot_results(datas, "results/wheel_controller/")

    datas = [
        loadmat(f"results/wheel_controller/data_combined_0.mat"),
        loadmat(f"results/wheel_controller/data_only_mf_{mf}.mat"),
        loadmat(f"results/wheel_controller/data_combined_{mf}.mat"),
    ]

    t = datas[0]["t"].flatten()
    fig, ax = plt.subplots(1,1,num="Front Left",clear=True)
    ax.plot(t,np.rad2deg(datas[0]["steerrefs"][:,0]),label=r"$\delta_w^r$")

    ax.plot(t,np.rad2deg(datas[0]["steers"][:,0]),label="PID")
    ax.plot(t,np.rad2deg(datas[1]["steers"][:,0]),label=r"FF")
    ax.plot(t,np.rad2deg(datas[2]["steers"][:,0]),label=r"PID+FF")


    ax.set_xlim(t[0],t[~0])
    #ax.set_yticks([90,0,-90,-180])
    ax.set_ylim(-10,10)
    #ax.legend(loc="lower left")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[deg]")
    ax.legend()

    fig.savefig("results/wheel_controller_comparison/comparison_fl.pdf")

    ax.legend(loc="upper left")
    ax.set_xlim(5,30)
    ax.set_ylim(-2,8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[deg]")
    fig.savefig("results/wheel_controller_comparison/comparison_zoom_start.pdf")

    ax.legend(loc="lower left")
    ax.set_xlim(100,101)
    ax.set_ylim(4.95,5.15)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[deg]")
    fig.savefig("results/wheel_controller_comparison/comparison_zoom.pdf")

    fig, ax = plt.subplots(1,1,num="Front Left tau_s",clear=True)
    ax.plot([],[]) # empty plot to get the same colors as above
    ax.plot([],[]) # empty plot to get the same colors as above
    ax.plot(t,datas[0]["steertorques"][:,0], label="PID")
    ax.plot(t,datas[1]["steertorques"][:,0], label="FF")
    ax.plot(t,datas[2]["steertorques"][:,0], label="PID+FF")

    ax.set_xlim(30,31)
    ax.ticklabel_format(style="sci")
    #ax.set_yticks([90,0,-90,-180])
    #ax.set_ylim(-50000,50000)
    ax.legend(loc="upper left")
    ax.set_xlabel("time [s]")
    #ax.set_ylabel("[deg]")

    fig.savefig("results/wheel_controller_comparison/steer_torque_fl.pdf")

    # Compare Kmf=1 with combined Kmf=0.05
    data_ff, = loadmat(f"results/wheel_controller/data_only_mf_1.mat"),
    data_comb, = loadmat(f"results/wheel_controller/data_combined_0.05.mat"),
    ref = data_ff["steerrefs"][:,0]
    t = data_ff["t"].flatten()
    fig, ax = plt.subplots(1,1,num="Front Left",clear=True)
    ax.plot(t,np.rad2deg(ref),label=r"$\delta_w^r$")

    labelff = f"FF, $K_{{mf}} = {data_ff['mf_factor'].item()}$"
    labelcomb = f"PID+FF, $K_{{mf}} = {data_comb['mf_factor'].item()}$"
    ax.plot(t,np.rad2deg(data_ff["steers"][:,0]),label=labelff)
    ax.plot(t,np.rad2deg(data_comb["steers"][:,0]),label=labelcomb)

    ax.set_xlim(t[0],t[~0])
    #ax.set_yticks([90,0,-90,-180])
    ax.set_ylim(-10,10)
    #ax.legend(loc="lower left")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[deg]")
    ax.legend()

    fig.savefig("results/wheel_controller_comparison/comparison_ff_vs_pidff.pdf")

    ax.legend(loc="upper left")
    ax.set_xlim(5,40)
    ax.set_ylim(-2,8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[deg]")
    fig.savefig("results/wheel_controller_comparison/comparison_zoom_start_ff_vs_pidff.pdf")

    ax.legend()
    ax.set_xlim(100,101)
    ax.set_ylim(4.95,5.15)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[deg]")
    fig.savefig("results/wheel_controller_comparison/comparison_zoom_ff_vs_pidff.pdf")

    fig, ax = plt.subplots(1,1,num="Front Left tau_s",clear=True)
    ax.plot([],[]) # empty plot to get the same colors as above
    ax.plot([],[]) # empty plot to get the same colors as above
    ax.plot(t,data_ff["steertorques"][:,0], label=labelff)
    ax.plot(t,data_comb["steertorques"][:,0], label=labelcomb)

    ax.set_xlim(30,31)
    ax.ticklabel_format(style="sci")
    #ax.set_yticks([90,0,-90,-180])
    #ax.set_ylim(-50000,50000)
    ax.legend(loc="upper left")
    ax.set_xlabel("time [s]")
    ax.set_ylim(-600,1200)
    #ax.set_ylabel("[deg]")

    fig.savefig("results/wheel_controller_comparison/steer_torque_fl_ff_vs_pidff.pdf")








