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
    #model.set([f"w{n}.B" for n in "1234"], [0.05]*4)
    inertia_steer, inertia_spin = model.get(["w1.inertia_steer", "w1.inertia_spin"])

    print(pos_in)
    print(vel_b)
    print(yaw)
    print(inertia_steer, inertia_spin)

    # real time plotting
    fig, ax_live = plt.subplots(1,1)
    pos_ref = np.array([5.0, 13.0])
    yaw_ref = 0.5
    ref_arrow = ax_live.arrow(0, 0, 2, 0, head_width=0.4, head_length=0.4)

    doliveplot = True
    timenextliveplotupdate = 0.0
    liveplotfps = 30
    animation = liveplot.VehicleAnimation(pos_in, length, width, size=50, ax=ax_live)

    t, dt, tstop = 0, 0.01, 10
    rows = []
    while t < tstop:
        # get data
        # steer_angles = model.get([f"w{n}.steer_angle" for n in "1234"])

        # set inputs
        model.setVec("drive_torques", [0, 0, 0, 0])
        model.setVec("steer_torques", [0, 0, 0, 0])
        
        # step simulation forward
        model.step(t, dt)
        t += dt

        # store results
        pos_in = model.getVec("body.pos_in", 3)
        vel_b = model.getVec("body.vel_b", 3)
        yaw, = np.deg2rad(model.get(["body.yaw"]))
        torque, = model.get(["body.torque[3]"])
        force = model.getVec("body.force_b", 3)
        steer_angles = np.deg2rad(model.get([f"w{n}.steer_angle" for n in "1234"]))
        wheel_alphas = np.deg2rad(model.get([f"w{n}.alpha" for n in "1234"]))
        wheel_slipress = model.get([f"w{n}.slip_res" for n in "1234"])
        wheel_slipls = model.get([f"w{n}.slip_l" for n in "1234"])
        wheel_slipss = model.get([f"w{n}.slip_s" for n in "1234"])
        wheel_slipl_ratios = model.get([f"w{n}.slip_l_ratio" for n in "1234"])
        wheel_slips_ratios = model.get([f"w{n}.slip_s_ratio" for n in "1234"])
        wheel_mures = model.get([f"w{n}.mu_res" for n in "1234"])
        wheel_vel_diff_l = model.get([f"w{n}.vel_diff_l" for n in "1234"])
        x, y = pos_in[0], pos_in[1]
        rows.append([t, x, y, yaw, torque, 
            *steer_angles, *wheel_alphas, *wheel_slipls, *wheel_slipss, *wheel_slipress, 
            *wheel_mures, *wheel_vel_diff_l])


        # debug
        velbigs = model.get([f"w{n}.vel_big" for n in "1234"])
        betas = model.get([f"w{n}.beta" for n in "1234"])
        omegas = model.get([f"w{n}.omega" for n in "1234"])
        w1_force_w_wheel = model.getVec("w1.force_w_wheel", 3)
        w1_vel_diff_l, w1_vel_diff_s = model.get(["w1.vel_diff_l", "w1.vel_diff_s"])
        driveStates = model.get([f"w{n}.driveState" for n in "1234"])
        # print(driveStates)


        # update live plot
        if doliveplot and t > timenextliveplotupdate:
            ref_trans = mpl.transforms.Affine2D().translate(pos_ref[0], pos_ref[1])
            ref_rot = mpl.transforms.Affine2D().rotate(yaw_ref)
            ref_arrow.set_transform(ref_rot + ref_trans + ax_live.transData)
            animation.update(t, pos_in, yaw, steer_angles)
            timenextliveplotupdate = t + 1/liveplotfps

    # draw last frame
    animation.update(tstop, pos_in, yaw, steer_angles)
    
    # results
    result = np.array(rows)
    ts = result[:,0]
    xs = result[:,1]
    ys = result[:,2]
    yaws = result[:,3]
    torques = result[:,4]
    steer_angles = result[:,5:9]
    wheel_alphas = result[:,9:13]
    wheel_slipls = result[:,13:17]
    wheel_slipss = result[:,17:21]
    wheel_slipress = result[:,21:25]
    wheel_mures = result[:,25:29]
    wheel_vel_diff_ls = result[:,29:33]

    fig1, axs = plt.subplots(2,2)
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
    fig1.tight_layout()

    fig2, axs2 = plt.subplots(4,1, sharex=True)
    axs2[0].plot(ts,steer_angles[:,0], label=r"$\delta_S$")
    axs2[0].plot(ts,wheel_alphas[:,0], label=r"$\alpha_W$")
    axs2[0].plot(ts,wheel_slipls[:,0], label=r"$s_{res}$")
    axs2[0].plot(ts,wheel_mures[:,0], label=r"$\mu_{res}$")
    axs2[0].set_ylabel("Wheel 1")
    axs2[0].legend()
    axs2[1].plot(ts,steer_angles[:,1], label=r"$\delta_S$")
    axs2[1].plot(ts,wheel_alphas[:,1], label=r"$\alpha_W$")
    axs2[1].plot(ts,wheel_slipls[:,1], label=r"$s_{res}$")
    axs2[1].plot(ts,wheel_mures[:,1], label=r"$\mu_{res}$")
    axs2[1].set_ylabel("Wheel 2")
    axs2[1].legend()
    axs2[2].plot(ts,steer_angles[:,2], label=r"$\delta_S$")
    axs2[2].plot(ts,wheel_alphas[:,2], label=r"$\alpha_W$")
    axs2[2].plot(ts,wheel_slipls[:,2], label=r"$s_{res}$")
    axs2[2].plot(ts,wheel_mures[:,2], label=r"$\mu_{res}$")
    axs2[2].set_ylabel("Wheel 3")
    axs2[2].legend()
    axs2[3].plot(ts,steer_angles[:,3], label=r"$\delta_S$")
    axs2[3].plot(ts,wheel_alphas[:,3], label=r"$\alpha_W$")
    axs2[3].plot(ts,wheel_slipls[:,3], label=r"$s_{res}$")
    axs2[3].plot(ts,wheel_mures[:,3], label=r"$\mu_{res}$")
    axs2[3].legend()
    axs2[3].set_xlabel("t")
    axs2[3].set_ylabel("Wheel 4")
    fig2.tight_layout()

    fig3, axs3 = plt.subplots(1,1, sharex=True)
    axs3.plot(ts,wheel_vel_diff_ls[:,0], label=r"$\Delta v$")
    axs3.plot(ts,wheel_slipls[:,0], label=r"$s_l$")
    axs3.set_ylabel("Wheel 1")
    axs3.legend()
    # axs3[1].plot(ts,wheel_vel_diff_ls[:,1], label=r"$\Delta v$")
    # axs3[1].plot(ts,wheel_slipls[:,1], label=r"$s_l$")
    # axs3[1].set_ylabel("Wheel 2")
    # axs3[1].legend()
    # axs3[2].plot(ts,wheel_vel_diff_ls[:,2], label=r"$\Delta v$")
    # axs3[2].plot(ts,wheel_slipls[:,2], label=r"$s_l$")
    # axs3[2].set_ylabel("Wheel 3")
    # axs3[2].legend()
    # axs3[3].plot(ts,wheel_vel_diff_ls[:,3], label=r"$\Delta v$")
    # axs3[3].plot(ts,wheel_slipls[:,3], label=r"$s_l$")
    # axs3[3].legend()
    # axs3[3].set_xlabel("t")
    # axs3[3].set_ylabel("Wheel 4")
    fig3.tight_layout()


    plt.show()


def should_rebuild(fmufile, mofiles):
    import os
    fmutime = os.path.getmtime(fmufile)
    mofiletime = max(list(map(os.path.getmtime, mofiles)))
    mofileisnewer = (mofiletime > fmutime)

    return mofileisnewer


if __name__=="__main__":
    mofiles = [
            "modelica/AutoagriModel.mo",
            "modelica/Body.mo",
            "modelica/Wheel.mo"]
    fmufile = "modelica/build/AutoagriModel.fmu"

    fmuready = False
    if should_rebuild(fmufile, mofiles):
        print("Model file updated, rebuilding FMU")
        from fmubuilder import build_autoagri
        fmuready = build_autoagri("modelica/build")
    else:
        fmuready = True

    if fmuready:
        autoagrimodel()
    else:
        print("fmu is not ready")
