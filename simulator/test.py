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
import rotations as R
from pid import PID

np.random.seed(1)
g = -10

window = window.Window("3D view", 1000,800)
renderer = RendererCollection(program.Program(shader.COLOR_SHADERS))

line_program = program.Program(shader.LINE_SHADERS)
plane = HorizontalPlaneRenderer(25, line_program)

# Setup world
world = ode.World()
world.setGravity( (0,0,g) )
world.setERP(0.8)
world.setCFM(1E-5)
space = ode.Space()


# Create objects and joints
ground = Plane((0,0,1), 0, world, space)


# wide wheeled motorcycle (so balance isn't an issue)
lx, ly, lz = 4.110, 2.440, 1
wheel_clearing = 1
r, h = 0.4, 0.7
height_above_ground = 0.2
zc = height_above_ground + lz/2 + r + wheel_clearing
chassis = Box(50, (lx,ly,lz), (0,0,zc), (0,0,0), world, space)

k = 0.0
zw = height_above_ground + r
rpy = (-np.pi/2,0,0)
wheel_front_left = Cylinder(50, r, h, (lx/2, ly/2, zw), rpy, world, space)
wheel_front_right = Cylinder(50, r, h, (lx/2, -ly/2, zw), rpy, world, space)
wheel_rear_left = Cylinder(50, r, h, (-lx/2, ly/2, zw), rpy, world, space)
wheel_rear_right = Cylinder(50, r, h, (-lx/2, -ly/2, zw), rpy, world, space)
 
def connect_wheel_to_chassis(wheel, chassis, world):
    joint = ode.Hinge2Joint(world)
    joint.attach(chassis.body, wheel.body)
    joint.setAnchor(wheel.position)
    joint.setAxis1((0,0,1)) # axis 1 is specified relative to body 1
    joint.setAxis2((0,1,0)) # axis 2 is specified relative to body 2
    return joint

joint_front_left = connect_wheel_to_chassis(wheel_front_left, chassis, world)
joint_front_right = connect_wheel_to_chassis(wheel_front_right, chassis, world)
joint_rear_left = connect_wheel_to_chassis(wheel_rear_left, chassis, world)
joint_rear_right = connect_wheel_to_chassis(wheel_rear_right, chassis, world)


vehicle_mass = chassis.mass.mass + 2*wheel_front_left.mass.mass + 2*wheel_rear_left.mass.mass
mu_ground = 0.8
friction_force_limit = abs(mu_ground*vehicle_mass*g)

print("friction_force_limit", friction_force_limit)


boxes = [
    chassis
]
for i in range(0):
    ls = np.random.uniform(0.1, 0.2, (3,))
    pos = np.random.uniform([-10,-10,0.5],[10,10,1])
    rpy = np.random.uniform(-np.pi, np.pi, (3,))
    
    b = Box(0, ls, pos, rpy, world, space)
    boxes.append(b)


cylinders = [
        wheel_front_left,
        wheel_front_right,
        wheel_rear_left,
        wheel_rear_right,
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

        # From manual:
        # To model [road-tire-interaction] in ODE set the tire-road contact parameters
        # as follows: 
        # set friction direction 1 in the direction that
        # the tire is rolling in, 
        # and set the FDS slip coefficient in friction direction 2 to kv, 
        # where v is the tire rolling velocity and k is a tire parameter 
        # that you can chose based on experimentation.
        #c.setFDir1(

        # note: mu is a force limit and not the Coulomb friction coefficient
        # http://ode.org/wiki/index.php?title=Manual#Contact
        c.setMu(friction_force_limit) 
        c.setMu2(friction_force_limit) # 
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

drive_fl_pid = PID(20,0,5)
drive_fr_pid = PID(20,0,5)
steer_fl_pid = PID(20,10,5)
steer_fr_pid = PID(20,10,5)

fps = 50
t, dt, tstop = 0, 1/fps, float('inf')
while t < tstop and not window.shouldClose():
    # setup drawing
    window.clear()

    vec_string = lambda p: f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}"

    omega_fl = joint_front_left.getAngle2Rate()
    steer_fl, steerrate_fl = joint_front_left.getAngle1(), joint_front_left.getAngle1Rate()
    omega_fr = joint_front_right.getAngle2Rate()
    steer_fr, steerrate_fr = joint_front_right.getAngle1(), joint_front_right.getAngle1Rate()

    angle_f = 2
    omega_fd = 2
    drive_fl = drive_fl_pid(dt, omega_fd - omega_fl)
    drive_fr = drive_fr_pid(dt, omega_fd - omega_fr)
    tz_fl = steer_fl_pid(dt, angle_f - steer_fl, -steerrate_fl)
    tz_fr = steer_fr_pid(dt, angle_f - steer_fr, -steerrate_fr)

    fx = 0
    joint_front_left.addTorques(tz_fl,drive_fl)
    joint_front_right.addTorques(tz_fr,fx)
    joint_rear_left.addTorques(0,fx)
    joint_rear_right.addTorques(0,fx)

    posx,posy,posz = chassis.position
    print(posx, posy, omega_fl)

    rc = 15
    cx = rc*np.cos(0.15*t)
    cy = rc*np.sin(0.14*t)
    eye = glm.vec3(posx+cx,posy+cy,rc/2)
    target = glm.vec3(posx,posy,posz)
    view = glm.lookAt(eye, target, glm.vec3(0,0,1))
    proj = glm.perspective(glm.radians(80), window.width/window.height, 0.01, 1000.0)
    # proj = glm.ortho(-20, 20, -20, 20, 0.01, 100)
    projview = proj*view
    renderer.draw(projview)
    plane.draw(projview)

    # step simulation with substepping
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
