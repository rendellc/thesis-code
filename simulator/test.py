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



# create vehicle
lx, ly, lz = 4, 2.5, 2
chassis = Box(50, (lx,ly,lz), (0,0,1.5), (0,0,0), world, space)

r, h, rpy = 0.5, 0.5, (np.pi/2, 0, 0)
wheel_fl = Cylinder(50, r, h, (lx/2, ly/2+h/2+r, r), rpy, world, space)
wheel_rl = Cylinder(50, r, h, (-lx/2, ly/2+h/2+r, r), rpy, world, space)
wheel_rr = Cylinder(50, r, h, (-lx/2, -ly/2-h/2-r, r), rpy, world, space)
wheel_fr = Cylinder(50, r, h, (lx/2, -ly/2-h/2-r, r), rpy, world, space)

def connect_chassis_wheel(chassis, wheel, world):
    joint = ode.UniversalJoint(world)
    joint.setAxis1((0,0,1))
    joint.setAxis2((0,1,0))
    joint.attach(wheel.body, chassis.body)

    body_pos_in_wheel = wheel.body.getPosRelPoint(chassis.position)
    print(chassis.position, wheel.position, body_pos_in_wheel)

    joint.setAnchor(body_pos_in_wheel)

    return joint

joint_fl = connect_chassis_wheel(chassis, wheel_fl, world)
joint_rl = connect_chassis_wheel(chassis, wheel_rl, world)
joint_rr = connect_chassis_wheel(chassis, wheel_rr, world)
joint_fr = connect_chassis_wheel(chassis, wheel_fr, world)



boxes = [
    chassis
]
for i in range(0):
    ls = np.random.uniform(0.1, 0.2, (3,))
    pos = np.random.uniform([-10,-10,0.5],[10,10,1])
    rpy = np.random.uniform(-np.pi, np.pi, (3,))
    
    b = Box(0, ls, pos, rpy, world, space)
    br = BoxRenderer(b)
    boxes.append(b)


cylinders = [
        #Cylinder(200, 0.4, 3, (15,0,1), (1.57,0,0), world, space)
        wheel_fl, wheel_rl, wheel_rr, wheel_fr
]
for i in range(0):
    r = 1 #np.random.uniform(0.5, 5)
    h = 3 # np.random.uniform(0.5, 5)
    pos = 5 * np.random.randn(3) + np.array([0,0,15])
    rpy = np.random.uniform(-np.pi, np.pi, (3,))
    c = Cylinder(50, r, h, pos, rpy, world, space)
    cr = CylinderRenderer(c)
    cylinders.append(c)

box_renderers = [BoxRenderer(b) for b in boxes]
cylinder_renderers = [CylinderRenderer(c) for c in cylinders]
renderer.add(*box_renderers, *cylinder_renderers)

# setup for collision detection
contactgroup = ode.JointGroup()
def near_callback(args, geom1, geom2):
    # check if objects collide
    contacts = ode.collide(geom1, geom2)
    world, contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(0.8)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

fps = 50
t, dt, tstop = 0, 1/fps, float('inf')
while t < tstop and not window.shouldClose():
    # setup drawing
    window.clear()

    #pos_string = lambda p: f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}"
    #print(pos_string(box1.body.getPosition()))
    #chassis.body.addRelForce((1000,0,0))
    joint_fl.addTorques(300,0)
    joint_rl.addTorques(300,0)
    joint_rr.addTorques(300,0)
    joint_fr.addTorques(300,0)

    rc = 15
    cx = rc*np.cos(0.1*t + np.pi/4)
    cy = rc*np.sin(0.1*t + np.pi/4)
    eye = glm.vec3(cx,cy,rc/2)
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
