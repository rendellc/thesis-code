import pybullet as p
import time
import pybullet_data

pc = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,2]
cubeStartOrientation = p.getQuaternionFromEuler([0.7,0,0.7])
vehicleId = p.loadURDF("models/vehicle.urdf", cubeStartPos, cubeStartOrientation)

numJoints = p.getNumJoints(vehicleId)
for i in range(numJoints):
    print(p.getJointInfo(vehicleId, i))

for i in range(10000):
    # cubePos, cubeOrn = p.getBasePositionAndOrientation(vehicleId)
    # cubeRpy = p.getEulerFromQuaternion(cubeOrn)
    p.setJointMotorControlArray(vehicleId, [3], p.TORQUE_CONTROL, forces=[1000])

    p.stepSimulation()
    time.sleep(1/240)

cubePos, cubeOrn = p.getBasePositionAndOrientation(vehicleId)
print(cubePos, cubeOrn)
p.disconnect()

