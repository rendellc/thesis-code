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

def burckhardt(s):
    return 1.2801*(1 - np.exp(-23.99*s)) - 0.52*s

def run_experiment(mf_factor,steer_poles,steer_reference,omega_all_reference, drive_pid):
    sim = VehicleSim(vp, sp)

    # Setup to save data
    fps = 30
    tstart, dt, tstop = 0, 1/fps, 600
    #tstart, dt, tstop = 0, 1/fps, 5
    num_steps = int((tstop - tstart)/dt)
    ts = np.zeros(num_steps)
    poss = np.zeros((num_steps,3))
    vels = np.zeros((num_steps,3))
    rpys = np.zeros((num_steps,3))
    rpyrates = np.zeros((num_steps,3))
    omegass = np.zeros((num_steps,4))
    omegarefs = np.zeros_like(omegass)
    steerss = np.zeros((num_steps,4))
    steerrefs = np.zeros_like(steerss)
    steerratess = np.zeros((num_steps,4))

    slip_long = np.zeros((num_steps,4))
    wheel_arms = sim.getWheelPositions()

    # Controller setup
    drive_torques = np.zeros(4)
    steer_torques = np.zeros((num_steps,4))
    inertia_steer = 0.25*wheel_mass*(wheel_radius**2 + 1/3*wheel_width**2)
    inertia_speed = 0.5*wheel_mass*wheel_radius**2

    #steer_poles = inertia_steer*steer_poles_no_inertia # desired poles of system, can be complex
    #print("Inertia steer:", inertia_steer)
    #print("Steer PID:", steer_poles[1],steer_poles[2],steer_poles[3])
    kd = steer_poles[2]
    kp = steer_poles[1]
    ki = steer_poles[0]
    print("Steer PID:", kp,ki,kd)

    kp_d = drive_pid[0] 
    ki_d = drive_pid[1] 
    kd_d = drive_pid[2] 
    print("Drive PID:", kp_d, ki_d, kd_d)

    class DriveController:
        def __init__(self, kp, ki, kd):
            self.pid = PID(kp,ki,kd)

        def __call__(self, dt, err, derr=None):
            return self.pid(dt, err, derr)

    drive_ctrls = [DriveController(kp_d,ki_d,kd_d) for _ in range(4)]
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
        slip_long[step_index] = wheel_slips_lo



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
        omega_refs = omega_all_reference(t)*np.array([1,1,1,1])
        steer_angle = steer_reference(t)
        steer_refs = np.deg2rad(steer_angle)*np.array([1,-1,-1,1])
        steer_error = ssa(steers - steer_refs)

        omegarefs[step_index] = omega_refs
        steerrefs[step_index] = steer_refs

        loads = -sim.getWheelLoads() # negative sign to turn into normal forces
        sign = np.sign
        #sign = lambda x: np.pi/2*np.arctan(x/0.001)
        steer_resistance = 1/3 * sign(steer_error)*wheel_width*1*loads

        mu_ls = burckhardt(np.abs(slip_long[step_index]))
        Ff_Ls = np.sign(slip_long[step_index])*mu_ls*loads
        #print(mu_ls,loads)
        #print(Ff_Ls)

        for i in range(4):
            # TODO: should also rotate to sx
            kf = 0
            #drive_torques[i] = kf*wheel_radius*Ff_Ls[i] + drive_ctrls[i](dt, omegas[i] - omega_refs[i])
            drive_torques[i] = drive_ctrls[i](dt, omegas[i] - omega_refs[i])

            steer_torques[step_index,i] = mf_factor * steer_resistance[i] + steer_pids[i](dt, steer_error[i], steerrates[i])

        # Apply control input
        sim.setTorques(steer_torques[step_index], drive_torques)
        shouldStop = sim.step(t, dt)


        # Setup for next iteration
        t += dt
        step_index += 1


    
    datadict = dict(
            t=ts,
            pos=poss,
            vel=vels,
            rpy=rpys,
            rpyrate=rpyrates,
            omegas=omegass,
            omegarefs=omegarefs,
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


def plot_results(matfile, postfix):
    data = loadmat(matfile)
    t = data["t"].flatten()

    for i in range(4):
        ij = data["order"][i]
        fig, ax = plt.subplots(1,1,num=ij,clear=True)
        ax.plot(t,np.rad2deg(data["omegarefs"][:,i]),label=rf"$\omega_{{{ij}}}^r$")
        ax.plot(t,np.rad2deg(data["omegas"][:,i]),label=rf"$\omega_{{{ij}}}$")

        ax.set_xlim(1,t[~0])
        #ax.set_yticks([90,0,-90,-180])
        #ax.set_ylim(-10,10)
        #ax.legend(loc="lower left")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("[deg/s]")
        ax.legend()
        fig.savefig(f"results/wheel_controller/omega_{ij}_{postfix}.pdf")

    fig, ax = plt.subplots(1,1,num="trajectory",clear=True)
    ax.plot(data["pos"][:,0],data["pos"][:,1])
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.savefig(f"results/wheel_controller/speed_trajectory_{postfix}.pdf")



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

    steering_reference = lambda t: 0

    #def omega_all_reference(t):
    #    if t < 10:
    #        return 0
    #    elif t < 20:
    #        return 1
    #    elif t < 30:
    #        return -1
    #    elif t < 60:
    #        return 1
    #    else:
    #        return 0
            
    kph = 5
    omega_ref = (kph/3.6)/wheel_radius
    # print(omega_ref)
    # print(np.rad2deg(omega_ref))

    omega_all_reference = lambda t: step(20,0,omega_ref)(t)

    #steer_poles = inertia_steer*np.poly1d([-2,-2+0.1j,-2-0.1j],True)
    # [_,kd,kp,ki]
    kmf = 0.05
    steer_poles = np.poly1d([1,200,4000,20])

    inertia_speed = 0.5*wheel_mass*wheel_radius**2
    kp_d = inertia_speed*15
    ki_d = inertia_speed*1
    kd_d = inertia_speed*5

    if False:
        # Combined
        data = run_experiment(kmf, steer_poles, lambda t: 0,omega_all_reference, (kp_d, 0, kd_d))
        savemat(f"results/wheel_controller/speed_straight_pd.mat", data)
        data = run_experiment(kmf, steer_poles, lambda t: 3,omega_all_reference, (kp_d, 0, kd_d))
        savemat(f"results/wheel_controller/speed_turn_pd.mat", data)
        data = run_experiment(kmf, steer_poles, lambda t: 0,omega_all_reference, (kp_d, ki_d, kd_d))
        savemat(f"results/wheel_controller/speed_straight_pid.mat", data)
        data = run_experiment(kmf, steer_poles, lambda t: 3,omega_all_reference, (kp_d, ki_d, kd_d))
        savemat(f"results/wheel_controller/speed_turn_pid.mat", data)

    plot_results(f"results/wheel_controller/speed_straight_pd.mat", "straight_pd")
    plot_results(f"results/wheel_controller/speed_turn_pd.mat", "turn_pd")
    plot_results(f"results/wheel_controller/speed_straight_pid.mat", "straight_pid")
    plot_results(f"results/wheel_controller/speed_turn_pid.mat", "turn_pid")


