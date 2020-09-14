from fmpy import dump, read_model_description, extract
from fmpy.fmi2 import FMU2Slave
from fmpy.util import plot_result

import numpy as np
import shutil
import matplotlib.pyplot as plt


fmu_filename = "TwoTrackVehicle.fmu"
start_time, time_step, stop_time = 0, 0.1, 30

# Setup FMU
model_description = read_model_description(fmu_filename)

dump(fmu_filename)

vrs = {}
for variable in model_description.modelVariables:
    vrs[variable.name] = variable.valueReference

print(vrs)

unzipdir = extract(fmu_filename)

fmu = FMU2Slave(guid=model_description.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=model_description.coSimulation.modelIdentifier,
        instanceName="instance1")

fmu.instantiate()
fmu.setupExperiment(startTime=start_time)
fmu.enterInitializationMode()
fmu.exitInitializationMode()


# pole-placement
zeta = 0.9
omega0 = 0.8
J = fmu.getReal([vrs["inertia"]])[0]
B = fmu.getReal([vrs["width"]])[0]
kp = 2*J*omega0**2/B
kd = 2*kp/omega0


t = start_time
rows = []
while t < stop_time:


    # Compute reference
    if t < stop_time/2:
        yaw_ref = 0
        yawrate_ref = 0
    else:
        yaw_ref = 3.0
        yawrate_ref = 0

    # Compute controller input
    yaw, yawrate = fmu.getReal([vrs["yaw"], vrs["yawrate"]])
    u_diff =  kp*(yaw - yaw_ref) + kd*(yawrate - yawrate_ref)
    u_sum = 0
    # u_sum = u_left+u_right
    # u_diff = u_left-u_right
    u_left = 0.5*(u_sum + u_diff)
    u_right = 0.5*(u_sum - u_diff)

    # Set input
    fmu.setReal([vrs["force_left"], vrs["force_right"]], [u_left, u_right])

    # do step
    fmu.doStep(currentCommunicationPoint=t, communicationStepSize=time_step)

    # get the values for 'outputs'
    x,y,yaw = fmu.getReal([vrs["x"], vrs["y"], vrs["yaw"]])

    # append results
    rows.append([t, x,y,yaw])

    # advance time
    t += time_step


fmu.terminate()
fmu.freeInstance()

# clean up
shutil.rmtree(unzipdir, ignore_errors=True)

# convert result to a structured numpy array
result = np.array(rows)

ts = result[:,0]
xs = result[:,1]
ys = result[:,2]
yaws = result[:,3]

plt.plot(ts,yaws)
plt.show()

