import ode

import rotations

import numpy as np

class Box:
    def __init__(self, mass, lengths, pos, rpy, world, space, doCollide=True):
        self.lengths = lengths
        
        self.body = ode.Body(world)
        self.mass = ode.Mass()
        self.mass.setBoxTotal(mass, *lengths)
        self.body.setMass(self.mass)

        if doCollide:
            self.geom = ode.GeomBox(space, self.lengths)
            self.geom.setBody(self.body)

        self.body.setPosition(pos)
        R = rotations.euler_to_matrix(rpy).flatten()
        self.body.setRotation(R)

    @property
    def position(self):
        return np.array(self.body.getPosition())

    @property
    def velocity(self):
        return np.array(self.body.getLinearVel())

    @property
    def rpy(self):
        rpy = rotations.quaternion_to_rpy(self.body.getQuaternion())
        return np.array(rpy)

    @property
    def rpyrate(self):
        return np.array(self.body.getAngularVel())


if __name__=="__main__":
    world = ode.World()
    world.setGravity( (0,0,-10) )
    world.setERP(0.8)
    world.setCFM(1E-5)
    space = ode.Space()
    box1 = Box(1, (2,1,0.5), (0,0,10), (0.7,1.3,-1), world, space)
    box2 = Box(1, (2,1,0.5), (0,0,10), (0.7,1.3,-1), world, space)


    print(box1)
    print(box2)

    settings = {
            box1: "settings for box 1",
            box2: "settings for box 2",
    }

    print(settings[box1])
    print(settings[box2])


