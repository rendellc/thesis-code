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
import glm

import ode
import time

from box import Box
from cylinder import Cylinder
from plane import Plane

import numpy as np

import window
from renderer import (
        RendererCollection,
        BoxRenderer,
        CylinderRenderer,
        HorizontalPlaneRenderer
)
import program
import shader

np.random.seed(1)

window = window.Window("3D view", 500,400)
renderer = RendererCollection(program.Program(shader.COLOR_SHADERS))

line_program = program.Program(shader.LINE_SHADERS)
plane = HorizontalPlaneRenderer(12, line_program)

# Setup world
world = ode.World()
world.setGravity( (0,0,-10) )
world.setERP(0.8)
world.setCFM(1E-5)
space = ode.Space()


# Create objects and joints
ground = Plane((0,0,1), 0, world, space)

boxes = []
box_renderers = []
for i in range(30):
    ls = np.random.uniform(0.1, 0.2, (3,))
    pos = np.random.uniform([-10,-10,0.5],[10,10,1])
    rpy = np.random.uniform(-np.pi, np.pi, (3,))
    
    b = Box(1, ls, pos, rpy, world, space)
    br = BoxRenderer(b)
    boxes.append(b)
    box_renderers.append(br)


cylinders = [
        Cylinder(100, 0.4, 3, (15,0,1), (1.57,0,0), world, space)
]
cylinder_renderers = [CylinderRenderer(c) for c in cylinders]
for i in range(0):
    r = 1 #np.random.uniform(0.5, 5)
    h = 3 # np.random.uniform(0.5, 5)
    pos = 5 * np.random.randn(3) + np.array([0,0,15])
    rpy = np.random.uniform(-np.pi, np.pi, (3,))
    c = Cylinder(50, r, h, pos, rpy, world, space)
    cr = CylinderRenderer(c)
    cylinders.append(c)
    cylinder_renderers.append(cr)

renderer.add(*box_renderers, *cylinder_renderers)

# setup for collision detection
contactgroup = ode.JointGroup()
def near_callback(args, geom1, geom2):
    # check if objects collide
    contacts = ode.collide(geom1, geom2)
    world, contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(30)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

fps = 50
t, dt, tstop = 0, 1/fps, float('inf')
while t < tstop and not window.shouldClose():
    # setup drawing
    window.clear()

    #pos_string = lambda p: f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}"
    #print(pos_string(box1.body.getPosition()))
    cylinders[0].body.addRelTorque((0,0,100))



    eye = glm.vec3(15,15,10)
    target = glm.vec3(0,0,0)
    view = glm.lookAt(eye, target, glm.vec3(0,0,1))
    proj = glm.perspective(glm.radians(80), window.width/window.height, 0.01, 1000.0)
    #proj = glm.ortho(-20, 20, -20, 20, 0.01, 100)
    projview = proj*view

    renderer.draw(projview)
    plane.draw(projview)

    # step simulation
    substeps = 2
    for i in range(substeps):
        space.collide((world, contactgroup), near_callback)
        world.step(dt/substeps)
        contactgroup.empty()
    t += dt

    window.swap()
    window.poll_events()
    time.sleep(dt)




window.terminate()
