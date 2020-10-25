"""
Script for testing simulator

Serves as documentation for how simulator should be used


From wiki:
A typical simulation will proceed like this:

    Create a dynamics world.
    Create bodies in the dynamics world.
    Set the state (position etc) of all bodies.
    Create joints in the dynamics world.
    Attach the joints to the bodies.
    Set the parameters of all joints.
    Create a collision world and collision geometry objects, as necessary.
    Create a joint group to hold the contact joints.
    Loop:
        Apply forces to the bodies as necessary.
        Adjust the joint parameters as necessary.
        Call collision detection.
        Create a contact joint for every collision point, and put it in the contact joint group.
        Take a simulation step.
        Remove all joints in the contact joint group.

    Destroy the dynamics and collision worlds.
"""

import ode
import time

from box import Box
from plane import Plane

from renderer import Renderer

renderer = Renderer()

# Setup world
world = ode.World()
world.setGravity( (0,0,-10) )
world.setERP(0.8)
world.setCFM(1E-5)
space = ode.Space()


# Create objects and joints
ground = Plane((0,0,1), 0, world, space)
box1 = Box(1, (2,1,0.5), (0,0,10), (0.7,1.3,-1), world, space)
box2 = Box(1, (2,1,0.5), (0,0,20), (0.7,1.3,-1), world, space)

renderer.add(ground)
#renderer.add(box1, box2)

# setup for collision detection
contactgroup = ode.JointGroup()
def near_callback(args, geom1, geom2):
    # check if objects collide
    contacts = ode.collide(geom1, geom2)
    world, contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(1)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

fps = 50
t, dt, tstop = 0, 1/fps, 10
while t < tstop:
    pos_string = lambda p: f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}"
    #print(pos_string(box1.body.getPosition()))

    # step simulation
    substeps = 2
    for i in range(substeps):
        space.collide((world, contactgroup), near_callback)
        world.step(dt/substeps)
        contactgroup.empty()
    t += dt

    #time.sleep(dt)
