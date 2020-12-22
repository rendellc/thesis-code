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
sp["do3Dview"] = True
sp["record3Dfilename"] = ""
sp["record3Dfps"] = 30
#sp["friction_scale"] = 0.01
wheel_radius = vp["wheel_radius"]


def generate_data():
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
    steerratess = np.zeros((num_steps,4))

    # Controller setup
    drive_torques = np.zeros(4)
    steer_torques = np.zeros(4)

    drive_pids = [PID(100,5,5) for _ in range(4)]
    steer_pids = [PID(250,20,10) for _ in range(4)]

    t = tstart
    tPltDraw, pltFps = t, 10
    shouldStop = False
    step_index = 0
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
        omega_all = -1
        if t > 30:
            omega_all = 1
        if t > 60:
            omega_all = -1
        if t > 90:
            omega_all = 1
        if t > 120:
            omega_all = -1

        omega_refs = omega_all*np.array([1,1,1,1])
        steer_refs = np.deg2rad(0)*np.array([1,-1,-1,1])

        for i in range(4):
            drive_torques[i] = drive_pids[i](dt, omegas[i] - omega_refs[i])
            steer_torques[i] = steer_pids[i](dt, ssa(steers[i] - steer_refs[i]), steerrates[i])


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
            wheel_arms=wheel_arms,
            order=["FL","RL","RR","FR"],
            videofile=sp["record3Dfilename"],
    )

    return datadict



def kinematic_models_test(data, figdir="./"):
    # Kinematic models
    ts = data["t"].flatten()
    wheel_arms = data["wheel_arms"]
    pos = data["pos"][0]
    vel = data["vel"][0]
    _,_,yaw = data["rpy"][0]
    num_steps = len(ts)

    # Full model
    A = np.zeros((8,3))
    zu = np.array([0,0,1])
    A[::2,0] = 1
    A[1::2,1] = 1
    A[:,2] = np.cross(zu,wheel_arms)[:,:2].flatten()
    Ainv = np.linalg.inv(A.T.dot(A)).dot(A.T)

    full_state = np.zeros((num_steps,3))
    full_state[0] = np.hstack([pos[:2],yaw]).flatten()
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


    # Slip calculation
    slip_long = np.zeros((num_steps,4))

    for i in range(len(ts)-1):
        pos = data["pos"][i]
        vel = data["vel"][i]
        rpy = data["rpy"][i]
        rpyrate = data["rpyrate"][i]
        omegas = data["omegas"][i]
        steers = data["steers"][i]
        vel_body = rot_z(-rpy[2]) @ vel
        dt = ts[i+1] - ts[i] # time to next step

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
        slip_long[i] = wheel_slips_lo

        # Kinematic model setup
        b[::2] = wheel_radius*omegas*np.cos(steers)
        b[1::2] = wheel_radius*omegas*np.sin(steers)

        # with slip modification in weight matrix
        if False:
            Winv = np.kron(np.diag(1/(1 - np.abs(wheel_slips_lo))), np.eye(2))
            Ainv = np.linalg.inv(A.T.dot(Winv).dot(A)).dot(A.T).dot(Winv)
            Axinv = Ax.T.dot(Winv)/(Ax.T.dot(Winv).dot(Ax))
            Ayinv = Ay.T.dot(Winv)/(Ay.T.dot(Winv).dot(Ay))
            Apinv = Ap.T.dot(Winv)/(Ap.T.dot(Winv).dot(Ap))
        #with slip modification in b (can be thought of as changing the effective radius of the wheel)
        # means the estimation meight be biased
        if False:
            b[0:2] = (1 - np.abs(wheel_slips_lo[0]))*b[0:2]
            b[2:4] = (1 - np.abs(wheel_slips_lo[0]))*b[2:4]
            b[4:6] = (1 - np.abs(wheel_slips_lo[0]))*b[4:6]
            b[6:8] = (1 - np.abs(wheel_slips_lo[0]))*b[6:8]

        # Full kinematic model
        ## compare with GT
        #full_error_pos = full_state[i,:2] - pos[:2]
        #full_error_yaw = abs(ssa(full_state[i,2] - rpy[2]))

        ## update for next timestep
        full_bodyrate = Ainv.dot(b) #[vx,vy,yawrate]
        full_J[:2,:2] = rot_z(full_state[i,2])[:2,:2]
        full_state[i+1] = full_state[i] + \
                dt*full_J.dot(full_bodyrate)

        # Simplified kinematic model
        ## compare with GT
        #simple_error_pos = simple_state[i,:2] - pos[:2]
        #simple_error_yaw = abs(ssa(simple_state[i,2] - rpy[2]))

        ## update for next timestep
        simple_bodyrate[0] = Axinv.dot(b)
        simple_bodyrate[1] = Ayinv.dot(b)
        simple_bodyrate[2] = Apinv.dot(b)
        simple_J[:2,:2] = rot_z(simple_state[i,2])[:2,:2]
        #simple_J[:2,:2] = rot_z(full_state[i,2])[:2,:2]
        simple_state[i+1] = simple_state[i] + \
                dt*simple_J.dot(simple_bodyrate)
        
        #se, fe = np.linalg.norm(simple_error_pos), np.linalg.norm(full_error_pos)
        #ratio = se/fe
        #print(f"{se:.5f} {fe:.5f} {ratio}")

    full_pos_error = full_state[:,:2] - data["pos"][:,:2]
    full_yaw_error = full_state[:,2] - data["rpy"][:,2]
    simple_pos_error = simple_state[:,:2] - data["pos"][:,:2]
    simple_yaw_error = simple_state[:,2] - data["rpy"][:,2]
    full_pos_error_norm = np.linalg.norm(full_pos_error, axis=1)
    full_yaw_error_norm = np.abs(ssa(full_yaw_error))
    simple_pos_error_norm = np.linalg.norm(simple_pos_error, axis=1)
    simple_yaw_error_norm = np.abs(ssa(simple_yaw_error))


    with np.errstate(divide="ignore",invalid="ignore"):
        error_ratio = simple_pos_error_norm/full_pos_error_norm
    error_ratio[np.isnan(error_ratio)] = 1

    fig, ax = plt.subplots(1,1,figsize=figsize,num="position error norm")
    ax.plot(ts, full_pos_error_norm, label="full")
    ax.plot(ts, simple_pos_error_norm, label="simple")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[m]")
    ax.set_xlim(ts[0], ts[~0])
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    fig.savefig(figdir + "pos_error.pdf")

    fig, ax = plt.subplots(1,1,figsize=figsize,num="yaw error norm")
    ax.plot(ts, np.rad2deg(full_yaw_error_norm), label="full")
    ax.plot(ts, np.rad2deg(simple_yaw_error_norm), label="simple")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("[deg]")
    ax.set_xlim(ts[0], ts[~0])
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    fig.savefig(figdir + "yaw_error.pdf")

    fig, ax = plt.subplots(1,1,figsize=figsize,num="ratio")
    ax.plot(ts, error_ratio)
    ax.set_xlim(ts[0], ts[~0])
    ax.set_ylim(bottom=0)
    ax.set_xlabel("time [s]")
    fig.savefig(figdir + "ratio.pdf")

    fig, ax = plt.subplots(1,1,figsize=figsize,num="trajectories")
    ax.plot(data["pos"][:,0],data["pos"][:,1], label="ground truth")
    ax.plot(full_state[:,0],full_state[:,1], label="full")
    ax.plot(simple_state[:,0],simple_state[:,1], label="simple")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="upper left")
    fig.savefig(figdir + "trajectories.pdf")


    fig, ax = plt.subplots(1,1,figsize=figsize,num="slip")
    for i in range(len(data["order"])):
        ax.plot(ts, slip_long[:,i], label=data["order"][i])
    ax.legend(loc="upper left")
    ax.set_xlim(ts[0], ts[~0])
    ax.set_ylim(-1,1)
    fig.savefig(figdir + "slip.pdf")
    #plt.show()
    

if __name__=="__main__":
    from scipy.io import savemat, loadmat
    from shutil import move

    data = generate_data()
    savemat("data/straight_then_reverse.mat", data)
    if data["videofile"]:
        move(data["videofile"], "data/straight_then_reverse.mp4")

    data = loadmat("data/straight_then_reverse.mat")
    kinematic_models_test(data, figdir="results/straight_then_reverse/")

