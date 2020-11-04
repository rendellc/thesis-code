import numpy as np
import matplotlib.pyplot as plt

from models.autoagri import createAutoagriModel

import simulator
import utils
import liveplots


model = createAutoagriModel(
        pos_in=[0,0,0],
        yaw=0,
        vel_b=[0,0,0],
        yawrate=0,
        wheel_angles=[0,0,0,0])

fig, axs = plt.subplots(2,2)
animationXY = liveplots.VehicleAnimation(model, size=20, ax=axs[0,0])
lineXY = liveplots.Plot([],[], ax=axs[0,0], label=r"$x$", rescale=False)
timeseriesOmega0 = liveplots.Plot([],[], ax=axs[1,0], label=r"$\omega_0$")
timeseriesOmega1 = liveplots.Plot([],[], ax=axs[1,0], label=r"$\omega_1$")
timeseriesOmega2 = liveplots.Plot([],[], ax=axs[1,0], label=r"$\omega_2$")
timeseriesOmega3 = liveplots.Plot([],[], ax=axs[1,0], label=r"$\omega_3$")

timeseriesDrive0 = liveplots.Plot([],[], ax=axs[1,1], label=r"$\tau_0$")
timeseriesDrive1 = liveplots.Plot([],[], ax=axs[1,1], label=r"$\tau_1$")
timeseriesDrive2 = liveplots.Plot([],[], ax=axs[1,1], label=r"$\tau_2$")
timeseriesDrive3 = liveplots.Plot([],[], ax=axs[1,1], label=r"$\tau_3$")

timeseriesDahlL0 = liveplots.Plot([],[], ax=axs[0,1], label=r"$F_{l0}$")
timeseriesDahlL1 = liveplots.Plot([],[], ax=axs[0,1], label=r"$F_{l1}$")
timeseriesDahlL2 = liveplots.Plot([],[], ax=axs[0,1], label=r"$F_{l2}$")
timeseriesDahlL3 = liveplots.Plot([],[], ax=axs[0,1], label=r"$F_{l3}$")

axs[0,1].legend(loc="upper left")
axs[1,0].legend(loc="upper left")
axs[1,0].set_title(r"$\omega_0$")
axs[1,1].set_title(r"$\tau_0$")
fig.tight_layout()


plt.show(block=False)

solver: simulator.Solver = simulator.BackwardEulerFsolve()
# solver: simulator.Solver = simulator.ImprovedEuler()

t, dt, tstop = 0, 0.1, 100
timenextliveupdate = t
liveplotfps = 30
omega_err_integral = np.array([0,0,0,0])
while t < tstop:
    omegas = np.array([w.omega for w in model.wheels])
    steer_angles = np.array([w.steer_angle for w in model.wheels])
    steer_angle_dots = np.array([w.steer_angle_dot for w in model.wheels])

    # steer angle PD, pole placement control
    omega0 = 1.1
    zeta = 0.9
    inertia_z = np.array([w.inertia_z  for w in model.wheels])
    kp_sa = inertia_z * (omega0**2)
    kd_sa = inertia_z * 2*zeta*omega0

    # omega PI, pole placement control
    omega_refs = np.array([0,0,0,0])
    inertia_y = np.array([w.inertia_y  for w in model.wheels])
    omega_err = omegas - omega_refs
    omega_err_integral = omega_err_integral + omega_err * dt
    omega0 = 1.1
    zeta = 0.9
    ki_o = inertia_y * (omega0**2)
    kp_o = inertia_y * 2*zeta*omega0
    drive_torques = -kp_o*omega_err - ki_o*omega_err_integral

    steer_angle_refs = np.array([0,0,0,0])

    drive_torques = np.array([100,100,100,100])
    if t > 5:
        drive_torques = np.array([0,0,0,0])

    # inputs = [drive1,steer1,...,drive4,steer4]
    steer_torques = -kp_sa*(steer_angles-steer_angle_refs) - kd_sa*steer_angle_dots
    inputs = utils.interleave([drive_torques, steer_torques])
    model = solver.step(model, inputs, dt)
    t += dt

    if t > timenextliveupdate:
        # print(wheel_states)
        steer_angles = [w.steer_angle for w in model.wheels]
        # print(model.wheels[0].omega)

        animationXY.update(t, model)
        timeseriesOmega0.update(t, model.wheels[0].omega)
        timeseriesOmega1.update(t, model.wheels[1].omega)
        timeseriesOmega2.update(t, model.wheels[2].omega)
        timeseriesOmega3.update(t, model.wheels[3].omega)
        timeseriesDahlL0.update(t, model.wheels[0].friction_dahl_l)
        timeseriesDahlL1.update(t, model.wheels[1].friction_dahl_l)
        timeseriesDahlL2.update(t, model.wheels[2].friction_dahl_l)
        timeseriesDahlL3.update(t, model.wheels[3].friction_dahl_l)

        timeseriesDrive0.update(t, drive_torques[0])
        timeseriesDrive1.update(t, drive_torques[1])
        timeseriesDrive2.update(t, drive_torques[2])
        timeseriesDrive3.update(t, drive_torques[3])

        lineXY.update(model.body.pos_in[0], model.body.pos_in[1])

        plt.pause(dt) # blit for better FPS

        timenextliveupdate = t + 1/liveplotfps


