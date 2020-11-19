import ode

import rotations

from cylinder import Cylinder

class Wheel:
    def __init__(self, massInner, radiusInner, massTire, radiusTire, height, pos, rpy, world, space):
        self.inner = Cylinder(massInner, radiusInner, height, pos, rpy, world, space)
        self.tire = Cylinder(massTire, radiusTire, 0.99*height, pos, rpy, world, space)

    @property
    def position(self):
        return self.inner.position

    @property
    def rpy(self):
        return self.inner.rpy

