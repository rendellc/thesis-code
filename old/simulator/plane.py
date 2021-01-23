import ode

class Plane:
    def __init__(self, normal, dist, world, space):
        """
        All x such that normal @ x = dist
        """

        self.plane = ode.GeomPlane(space, normal, dist)

