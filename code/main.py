from models import SimulatorModel

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import liveplot


def vehiclemodel():
    model = SimulatorModel(start_time=0, fmufilepath="modelica/build/VehicleForceModel.fmu")

    #model.set(["yaw"], [-1.57])
    model.setVec("pos_in", [0, 0, 0])

    pos_in = model.getVec("pos_in", 3)
    vel_b = model.getVec("vel_b", 3)
    yaw, = model.get(["yaw"])
    length, width = model.get(["length", "width"])

    print("pos_in:", pos_in)
    print("vel_b:", vel_b)
    print("yaw:", yaw)

    
    # real time plotting
    fig, ax_live = plt.subplots(1,1)
    pos_ref = np.array([5.0, 13.0])
    yaw_ref = 0.5
    dx,dy = np.cos(yaw_ref), np.sin(yaw_ref)
    ref_arrow = ax_live.arrow(0, 0, 2, 0, head_width=0.4, head_length=0.4)


    doliveplot = True
    if doliveplot:
        animation = liveplot.VehicleAnimation(pos_in, length, width, ax=ax_live)

    t, dt, tstop = 0, 0.1, 40
    rows = []
    forces = [0,0,0,0]
    while t < tstop:
        # set input
        if t < 10:
            forces = np.array([2,2,2,2])
        elif t < 30:
            forces = np.array([-2,0,0,-4])


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
        rows.append([t, x, y, yaw, torque, *forces])

        # update live plot
        if doliveplot:
            ref_trans = mpl.transforms.Affine2D().translate(pos_ref[0], pos_ref[1])
            ref_rot = mpl.transforms.Affine2D().rotate(yaw_ref)
            ref_arrow.set_transform(ref_rot + ref_trans + ax_live.transData)

            animation.update(pos_in, yaw, forces)

    
    result = np.array(rows)
    ts = result[:,0]
    xs = result[:,1]
    ys = result[:,2]
    yaws = result[:,3]
    torques = result[:,4]
    us = result[:,5:]

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
    axs[1,1].plot(ts,us[:,0], label="$u_{fl}$")
    axs[1,1].plot(ts,us[:,1], label="$u_{rl}$")
    axs[1,1].plot(ts,us[:,2], label="$u_{rr}$")
    axs[1,1].plot(ts,us[:,3], label="$u_{fr}$")
    axs[1,1].set_xlabel("t")
    axs[1,1].set_ylabel("$u$", rotation=0)
    axs[1,1].legend()

    fig.tight_layout()

    plt.show()






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


def linear2dmovement():
    model = SimulatorModel(start_time=0, fmufilepath="modelica/build/Linear2DMovement.fmu")

    x = model.getVec("x", 2)
    vx = model.getVec("vx", 2)

    print("x:", x)
    print("vx:", vx)

    mass, mu = model.get(["m","mu"])
    print("mass:", mass)
    print("friction:", mu)


    t, dt, tstop = 0, 0.1, 40
    rows = []
    forces = [0,0]
    while t < tstop:

        # set input
        if t< tstop/4:
            u = np.array([1,0])
        elif t < 2*tstop/4:
            u = np.array([-0.5,0.1])
        elif t < 3*tstop/4:
            u = np.array([-0.1,-0.1])

        model.setVec("u", u)
        
        # step simulation forward
        model.step(t, dt)
        t += dt

        # store results
        x = model.getVec("x", 2)
        vx = model.getVec("vx", 2)
        rows.append([t, *x, *u])

    
    result = np.array(rows)
    ts = result[:,0]
    xs = result[:,1]
    ys = result[:,2]
    us = result[:,3:]

    axs[0].plot(xs,ys)
    axs[0].set_xlabel("x")
    axs[0].set_ylabel("y", rotation=0)
    axs[1].plot(ts,us[:,0], label=r"$u_x$")
    axs[1].plot(ts,us[:,1], label=r"$u_y$")
    axs[1].set_xlabel("t")
    axs[1].set_ylabel(r"$u$", rotation=0)
    axs[1].legend()

    fig.tight_layout()

    plt.show()



if __name__=="__main__":
    #linear2dmovement()
    vehiclemodel()
    #twotrackmodel()
