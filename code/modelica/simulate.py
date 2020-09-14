from fmpy import dump, read_model_description, extract
from fmpy.fmi2 import FMU2Slave
from fmpy.util import plot_result

import numpy as np
import shutil
import matplotlib.pyplot as plt


fmu_filename = "TwoTrackVehicle.fmu"
start_time, time_step, stop_time = 0, 0.1, 20

# Setup FMU
model_description = read_model_description(fmu_filename)

dump(fmu_filename)

vrs = {}
for variable in model_description.modelVariables:
    vrs[variable.name] = variable.valueReference

print(vrs)
vr_x = vrs["x"]
vr_u = vrs["u"]

print("x", vr_x)
print("u", vr_u)

import sys
sys.exit(1)

unzipdir = extract(fmu_filename)

fmu = FMU2Slave(guid=model_description.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=model_description.coSimulation.modelIdentifier,
        instanceName="instance1")

fmu.instantiate()
fmu.setupExperiment(startTime=start_time)
fmu.enterInitializationMode()
fmu.exitInitializationMode()


# change paramaters
fmu.setReal([vrs["m"]], [0.1])

t = start_time
rows = []
while t < stop_time:

    # set input
    if t < stop_time/2:
        fmu.setReal([vr_u], [0])
    else:
        fmu.setReal([vr_u], [1])



    # do step
    fmu.doStep(currentCommunicationPoint=t, communicationStepSize=time_step)

    # get the values for 'outputs'
    x = fmu.getReal([vr_x])

    # append results
    rows.append([t, x[0]])

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

plt.plot(ts,xs)
plt.show()

