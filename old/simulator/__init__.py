#print(__name__, __file__,__path__)

import sys

for p in __path__:
    #print("adding", p)
    sys.path.insert(1, p)

from vehiclesim import VehicleSim


from box import Box
from cylinder import Cylinder
from plane import Plane

from renderer import (
        RendererCollection,
        BoxRenderer,
        CylinderRenderer,
        HorizontalPlaneRenderer
)
import program
import shader
