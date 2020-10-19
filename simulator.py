import pybullet as p
import pybullet_data

class Vehicle:
    def __init__(self, startPos, startRPY, gui=True):
        if gui:
            self.pc = p.connect(p.GUI)
        else:
            raise NotImplementedError("Non-gui simulator not implemented")
        
        # assert realtime, "Non-realtime simulation not implemented"
        # p.setRealTimeSimulation(int(realtime))

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        planeId = p.loadURDF("plane.urdf")
        pos0 = startPos
        orn0 = p.getQuaternionFromEuler(startRPY)
        self.vehicleId = p.loadURDF("models/vehicle.urdf", pos0, orn0)

        self.wheelDriveJoints = []
        numJoints = p.getNumJoints(self.vehicleId)
        for i in range(numJoints):
            info = p.getJointInfo(self.vehicleId, i)
            jointIndex = info[0]
            childName = info[12]
            if b"wheel_" in childName:
                self.wheelDriveJoints.append(jointIndex)

    def getPositionAndOrientation(self):
        pos, quat = p.getBasePositionAndOrientation(self.vehicleId)
        rpy = p.getEulerFromQuaternion(quat)
        return pos, rpy

    def setTorques(self, driveTorques):
        p.setJointMotorControlArray(self.vehicleId, self.wheelDriveJoints, p.TORQUE_CONTROL, forces=driveTorques)

    def stepSimulation(self):
        p.stepSimulation()


    def __del__(self):
        p.disconnect()

