#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

plt.style.use(["science","no-latex"])
#plt.style.use(["science","ieee","no-latex"])
np.set_printoptions(precision=4)

from simulator import VehicleSim
from simulator.common import PID
from simulator.common import ssa
from simulator.rotations import rot_z, rot_z_der
import liveplot
from blitmanager import BlitManager

import pyautogui
import cv2

import tqdm
import time

import params
vp = params.VEHICLE_PARAMS
sp = params.SIM_PARAMS
sp["do3Dview"] = True
wheel_radius = vp["wheel_radius"]


def generate_data():
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
    steerratess = np.zeros((num_steps,4))


    recordfile = ""
    if sp["record3D"]:
        recordfile = "3dview.mp4"
        codec = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(recordfile, codec, fps, (sp["window_width"],sp["window_height"]))


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

        if sp["record3D"]:
            frame = pyautogui.screenshot(region=(sim.window.x, sim.window.y, sim.window.width, sim.window.height))
            frame = np.array(frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            out.write(frame)

        # Controllers and References
        omega_all = 0
        if t > 5:
            omega_all = 1
        if t > 90:
            omega_all = -1
        if t > 150:
            omega_all = 1

        omega_refs = omega_all*np.array([1,1,1,1])

        if t < 30:
            steer_refs = np.deg2rad(5)*np.array([1,-1,-1,1])
        elif t < 60:
            steer_refs = np.deg2rad(0)*np.array([1,-1,-1,1])
        else:
            steer_refs = np.deg2rad(-5)*np.array([1,-1,-1,1])
        #else:
        #    steer_refs = np.deg2rad(0)*np.array([1,-1,-1,1])


        for i in range(4):
            drive_torques[i] = drive_pids[i](dt, omegas[i] - omega_refs[i])
            steer_torques[i] = steer_pids[i](dt, ssa(steers[i] - steer_refs[i]), steerrates[i])


        # Apply control input
        sim.setTorques(steer_torques, drive_torques)
        shouldStop = sim.step(t, dt)


        # Setup for next iteration
        t += dt
        step_index += 1

    if sp["record3D"]:
        out.release()
        cv2.destroyAllWindows()

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
            order="FL,RL,RR,FR",
            videofile=recordfile,
    )

    return datadict



def kinematic_models_test(data):
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



    for i in range(len(ts)-1):
        pos = data["pos"][i]
        rpy = data["rpy"][i]
        omegas = data["omegas"][i]
        steers = data["steers"][i]
        dt = ts[i+1] - ts[i] # time to next step

        # Kinematic model setup
        b[::2] = wheel_radius*omegas*np.cos(steers)
        b[1::2] = wheel_radius*omegas*np.sin(steers)

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

    fig, ax = plt.subplots(1,1,num="position error norm")
    ax.plot(ts, full_pos_error_norm, label="full")
    ax.plot(ts, simple_pos_error_norm, label="simple")
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"$|p - \hat{p}|$")
    ax.set_xlim(ts[0], ts[~0])
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    fig.savefig("results/kinematic_test/pos_error.pdf")

    fig, ax = plt.subplots(1,1,num="yaw error norm")
    ax.plot(ts, full_yaw_error_norm, label="full")
    ax.plot(ts, simple_yaw_error_norm, label="simple")
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"$|\psi - \hat{\psi}|$")
    ax.set_xlim(ts[0], ts[~0])
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    fig.savefig("results/kinematic_test/yaw_error.pdf")

    plt.show()
    

if __name__=="__main__":
    from scipy.io import savemat, loadmat
    from shutil import move

    # data = generate_data()
    # data["title"] = "Long run"
    # data["description"] = """
    #     Long run with turn, reverse motion, slip, and other unmodelled behaviour.
    # """
    # savemat("data/kinematic_test_long.mat", data)
    # if data["videofile"]:
    #     move(data["videofile"], "data/kinematic_test_long.mp4")

    data = loadmat("data/kinematic_test_long.mat")
    kinematic_models_test(data)

