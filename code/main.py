from models import SimulatorModel

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import liveplot


def autoagrimodel():
    model = SimulatorModel(start_time=0, fmufilepath="modelica/build/AutoagriModel.fmu", printVariableNames=False)

    pos_in = model.getVec("body.pos_in", 3)
    vel_b = model.getVec("body.vel_b", 3)
    yaw, = np.deg2rad(model.get(["body.yaw"]))
    length, width = model.get(["body.length", "body.width"])
    model.set([f"w{n}.B" for n in "1234"], [0.05]*4)

    print(pos_in)
    print(vel_b)
    print(yaw)
    print(length, width)

    # real time plotting
    fig, ax_live = plt.subplots(1,1)
    pos_ref = np.array([5.0, 13.0])
    yaw_ref = 0.5
    ref_arrow = ax_live.arrow(0, 0, 2, 0, head_width=0.4, head_length=0.4)

    doliveplot = True
    timenextliveplotupdate = 0.0
    liveplotfps = 60
    animation = liveplot.VehicleAnimation(pos_in, length, width, size=50, ax=ax_live)

    t, dt, tstop = 0, 0.01, 20
    rows = []
    while t < tstop:
        # get data
        # wheel_angles = model.get([f"w{n}.steer_angle" for n in "1234"])

        # set inputs
        model.setVec("drive_torques", [1, 0, 0, 1])
        model.setVec("steer_torques", [0, 0, 0, 0])
        
        # step simulation forward
        model.step(t, dt)
        t += dt

        # store results
        pos_in = model.getVec("body.pos_in", 3)
        vel_b = model.getVec("body.vel_b", 3)
        yaw, = np.deg2rad(model.get(["body.yaw"]))
        torque, = model.get(["body.torque[3]"])
        wheel_angles = np.deg2rad(model.get([f"w{n}.steer_angle" for n in "1234"]))
        wheel_sideslips = np.deg2rad(model.get([f"w{n}.sideslip" for n in "1234"]))
        wheel_slipls = model.get([f"w{n}.slip_l" for n in "1234"])
        wheel_mures = model.get([f"w{n}.mu_res" for n in "1234"])
        x, y = pos_in[0], pos_in[1]
        rows.append([t, x, y, yaw, torque, *wheel_angles, *wheel_sideslips, *wheel_slipls, *wheel_mures])

        # debug
        velbigs = model.get([f"w{n}.vel_big" for n in "1234"])
        omegas = model.get([f"w{n}.omega" for n in "1234"])
        print(velbigs)


        # update live plot
        if doliveplot and t > timenextliveplotupdate:
            ref_trans = mpl.transforms.Affine2D().translate(pos_ref[0], pos_ref[1])
            ref_rot = mpl.transforms.Affine2D().rotate(yaw_ref)
            ref_arrow.set_transform(ref_rot + ref_trans + ax_live.transData)
            animation.update(t, pos_in, yaw, wheel_angles)
            timenextliveplotupdate = t + 1/liveplotfps


    # draw last frame
    animation.update(tstop, pos_in, yaw, wheel_angles)
    
    result = np.array(rows)
    ts = result[:,0]
    xs = result[:,1]
    ys = result[:,2]
    yaws = result[:,3]
    torques = result[:,4]
    wheel_angles = result[:,5:9]
    wheel_sideslips = result[:,9:13]
    wheel_slipls = result[:,13:17]
    wheel_mures = result[:,17:21]

    fig, axs = plt.subplots(2,2)
    axs[0,0].plot(xs,ys)
    axs[0,0].set_xlabel("x")
    axs[0,0].set_ylabel("y", rotation=0)
    axs[0,1].plot(ts,yaws)
    axs[0,1].set_xlabel("t")
    axs[0,1].set_ylabel(r"$\psi$", rotation=0)
    axs[1,0].plot(ts, xs)
    axs[1,0].set_xlabel("t")
    axs[1,0].set_ylabel(r"$x$", rotation=0)
    axs[1,1].plot(ts, ys)
    axs[1,1].set_xlabel("t")
    axs[1,1].set_ylabel(r"$y$", rotation=0)
    fig.tight_layout()

    fig2, axs2 = plt.subplots(4,1, sharex=True)
    axs2[0].plot(ts,wheel_angles[:,0], label=r"$\delta_S$")
    axs2[0].plot(ts,wheel_sideslips[:,0], label=r"$\alpha_W$")
    axs2[0].plot(ts,wheel_slipls[:,0], label=r"$s_{res}$")
    axs2[0].plot(ts,wheel_mures[:,0], label=r"$\mu_{res}$")
    axs2[0].set_ylabel("Wheel 1")
    axs2[0].legend()
    axs2[1].plot(ts,wheel_angles[:,1], label=r"$\delta_S$")
    axs2[1].plot(ts,wheel_sideslips[:,1], label=r"$\alpha_W$")
    axs2[1].plot(ts,wheel_slipls[:,1], label=r"$s_{res}$")
    axs2[1].plot(ts,wheel_mures[:,1], label=r"$\mu_{res}$")
    axs2[1].set_ylabel("Wheel 2")
    axs2[1].legend()
    axs2[2].plot(ts,wheel_angles[:,2], label=r"$\delta_S$")
    axs2[2].plot(ts,wheel_sideslips[:,2], label=r"$\alpha_W$")
    axs2[2].plot(ts,wheel_slipls[:,2], label=r"$s_{res}$")
    axs2[2].plot(ts,wheel_mures[:,2], label=r"$\mu_{res}$")
    axs2[2].set_ylabel("Wheel 3")
    axs2[2].legend()
    axs2[3].plot(ts,wheel_angles[:,3], label=r"$\delta_S$")
    axs2[3].plot(ts,wheel_sideslips[:,3], label=r"$\alpha_W$")
    axs2[3].plot(ts,wheel_slipls[:,3], label=r"$s_{res}$")
    axs2[3].plot(ts,wheel_mures[:,3], label=r"$\mu_{res}$")
    axs2[3].legend()
    axs2[3].set_xlabel("t")
    axs2[3].set_ylabel("Wheel 4")

    fig2.tight_layout()


    plt.show()





if __name__=="__main__":
    autoagrimodel()
