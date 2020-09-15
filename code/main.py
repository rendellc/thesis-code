from models import SimulatorModel

import numpy as np
import matplotlib.pyplot as plt


def twotrackmodel():
    model = SimulatorModel(start_time=0, fmufilepath="modelica/build/TwoTrackVehicle.fmu")

    # pole-placement computations
    zeta = 0.9
    omega0 = 0.8
    J, B = model.get(["inertia", "width"])
    kp = 2*J*omega0**2/B
    kd = 2*kp/omega0


    yaw, = model.get(["yaw"])
    print("initial yaw:", yaw)
    model.set(["yaw"], [0])
    yaw, = model.get(["yaw"])
    print("initial yaw:", yaw)


    t, dt, tstop = 0, 0.1, 40
    rows = []
    while t < tstop:

        # decide reference
        if t < tstop/2:
            yaw_ref = 0
            yawrate_ref = 0
        else:
            yaw_ref = 3.0
            yawrate_ref = 0

        # compute controller
        yaw, yawrate = model.get(["yaw", "yawrate"])
        u_diff =  kp*(yaw - yaw_ref) + kd*(yawrate - yawrate_ref)
        u_sum = 1
        # u_sum = u_left+u_right
        # u_diff = u_left-u_right
        u_left = 0.5*(u_sum + u_diff)
        u_right = 0.5*(u_sum - u_diff)


        # set input
        model.set(["force_left", "force_right"], [u_left, u_right])
        
        # step simulation forward
        model.step(t, dt)
        t += dt

        # store results
        x,y,yaw = model.get(["x", "y", "yaw"])
        rows.append([t, x, y, yaw])

    
    result = np.array(rows)
    ts = result[:,0]
    xs = result[:,1]
    ys = result[:,2]
    yaws = result[:,3]

    print(ts.shape)
    print(xs.shape)
    print(ys.shape)
    print(yaws.shape)

    fig, axs = plt.subplots(1,2)
    axs[0].plot(xs,ys)
    axs[1].plot(ts,yaws)

    plt.show()


def vehiclemodel():
    model = SimulatorModel(start_time=0, fmufilepath="modelica/build/Vehicle.fmu")

    yaw, = model.get(["yaw"])
    print("initial yaw:", yaw)
    model.set(["yaw"], [0])
    yaw, = model.get(["yaw"])
    print("initial yaw:", yaw)

    pos_in = model.getVec("pos_in", 3)
    vel_b = model.getVec("vel_b", 3)

    print("pos_in:", pos_in)
    print("vel_b:", vel_b)


    t, dt, tstop = 0, 0.1, 40
    rows = []
    forces = [0,0,0,0]
    while t < tstop:

        # set input
        forces = np.array([t,0,0,20])/10

        model.setVec("forces", forces)
        
        # step simulation forward
        model.step(t, dt)
        t += dt

        # store results
        pos_in = model.getVec("pos_in", 3)
        vel_b = model.getVec("vel_b", 3)
        yaw, = model.get(["yaw"])
        torque, = model.get(["torque_total"])
        x, y = pos_in[0], pos_in[1]
        rows.append([t, x, y, yaw, torque])

    
    result = np.array(rows)
    ts = result[:,0]
    xs = result[:,1]
    ys = result[:,2]
    yaws = result[:,3]
    torques = result[:,4]

    fig, axs = plt.subplots(2,2)
    axs[0,0].plot(xs,ys)
    axs[0,0].set_xlabel("x")
    axs[0,0].set_ylabel("y", rotation=0)
    axs[0,1].plot(ts,yaws)
    axs[0,1].set_xlabel("t")
    axs[0,1].set_ylabel(r"$\psi$", rotation=0)
    axs[1,0].plot(ts,torques)
    axs[1,0].set_xlabel("t")
    axs[1,0].set_ylabel(r"$\tau$", rotation=0)

    fig.tight_layout()

    plt.show()




if __name__=="__main__":
    vehiclemodel()
    #twotrackmodel()
