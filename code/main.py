from models import SimulatorModel

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import liveplot


def autoagrimodel():
    model = SimulatorModel(start_time=0, fmufilepath="modelica/build/AutoagriModel.fmu", printVariableNames=True)

    pos_in = model.getVec("body.pos_in", 3)
    vel_b = model.getVec("body.vel_b", 3)
    yaw, = np.deg2rad(model.get(["body.yaw"]))
    length, width = model.get(["body.length", "body.width"])
    model.set(["w4.steer_angle"], [np.pi/2])

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
    animation = liveplot.VehicleAnimation(pos_in, length, width, ax=ax_live)

    t, dt, tstop = 0, 0.1, 20
    rows = []
    while t < tstop:
        # get data
        # wheel_angles = model.get([f"w{n}.steer_angle" for n in "1234"])

        # set inputs
        model.setVec("drive_torques", [0.0000001, 0, 0, 0])
        model.setVec("steer_torques", [0, 0, 0, 0.5])

        
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
        x, y = pos_in[0], pos_in[1]
        rows.append([t, x, y, yaw, torque, *wheel_angles, *wheel_sideslips])

        # update live plot
        if doliveplot:
            ref_trans = mpl.transforms.Affine2D().translate(pos_ref[0], pos_ref[1])
            ref_rot = mpl.transforms.Affine2D().rotate(yaw_ref)
            ref_arrow.set_transform(ref_rot + ref_trans + ax_live.transData)
            animation.update(pos_in, yaw, wheel_angles)


    # draw last frame
    animation.update(pos_in, yaw, wheel_angles)
    
    result = np.array(rows)
    ts = result[:,0]
    xs = result[:,1]
    ys = result[:,2]
    yaws = result[:,3]
    torques = result[:,4]
    wheel_angles = result[:,5:9]
    wheel_sideslips = result[:,9:13]

    fig, axs = plt.subplots(2,2)
    axs[0,0].plot(xs,ys)
    axs[0,0].set_xlabel("x")
    axs[0,0].set_ylabel("y", rotation=0)
    axs[0,1].plot(ts,yaws)
    axs[0,1].set_xlabel("t")
    axs[0,1].set_ylabel(r"$\psi$", rotation=0)
    axs[1,0].plot(ts, wheel_sideslips)
    axs[1,0].set_xlabel("t")
    axs[1,0].set_ylabel(r"$\alpha_W$", rotation=0)
    axs[1,1].plot(ts, wheel_angles)
    axs[1,1].set_xlabel("t")
    axs[1,1].set_ylabel(r"$\delta_S$", rotation=0)
    axs[1,1].legend()

    fig.tight_layout()

    plt.show()





if __name__=="__main__":
    autoagrimodel()
