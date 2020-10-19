from simulator import Vehicle
import pybullet as p


import time

vehicle = Vehicle([0,0,1.5], [0,0,0])


for i in range(10000):
    pos, rpy = vehicle.getPositionAndOrientation()

    x,y,z = pos
    yaw = rpy[2]
    print(f"{x:.2f} {y:.2f} {z:.2f}\t{yaw:.3f}")

    vehicle.setTorques([3000,2000,0,-3000])

    vehicle.stepSimulation()
    time.sleep(1/240)

cubePos, cubeOrn = p.getBasePositionAndOrientation(vehicleId)
print(cubePos, cubeOrn)
p.disconnect()

