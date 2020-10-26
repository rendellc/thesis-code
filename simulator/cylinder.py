import ode

import rotations

class Cylinder:
    def __init__(self, mass, radius, height, pos, rpy, world, space):
        self.radius = radius
        self.height = height
        
        self.body = ode.Body(world)
        self.mass = ode.Mass()
        self.mass.setCylinderTotal(mass, 3, radius, height)
        self.body.setMass(self.mass)

        self.geom = ode.GeomCylinder(space, radius, height)
        self.geom.setBody(self.body)

        self.body.setPosition(pos)
        R = rotations.euler_to_matrix(rpy).flatten()
        self.body.setRotation(R)

    @property
    def position(self):
        return self.body.getPosition()

    @property
    def rpy(self):
        rpy = rotations.quaternion_to_rpy(self.body.getQuaternion())
        return rpy


if __name__=="__main__":
    world = ode.World()
    world.setGravity( (0,0,-10) )
    world.setERP(0.8)
    world.setCFM(1E-5)
    space = ode.Space()
    c1 = Cylinder(1, 2, 0.5, (0,0,10), (0.7,1.3,-1), world, space)
    c2 = Cylinder(10, 0.5, 2, (0,0,10), (-0.7,-1.3,-1), world, space)

    print(c1)
    print(c1)

    settings = {
            c1: "settings for cylinder 1",
            c2: "settings for cylinder 2",
    }

    print(settings[c1])
    print(settings[c2])


