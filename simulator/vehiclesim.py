
import ode
import glm
import numpy as np


from box import Box
from cylinder import Cylinder
from plane import Plane

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



def connect_fixed(b1, b2, world):
    joint = ode.FixedJoint(world)
    joint.attach(b1.body, b2.body)
    joint.setFixed()
    return joint


def connect_wheel_to_body(wheel, body, world):
    joint = ode.Hinge2Joint(world)
    joint.attach(body.body, wheel.body)
    joint.setAnchor(wheel.position)
    joint.setAxis1((0,0,1)) # axis 1 is specified relative to body 1
    joint.setAxis2((0,1,0)) # axis 2 is specified relative to body 2
    return joint


class VehicleSim:
    def __init__(self, vehicle_params, sim_params):
        self.sim_params = sim_params
        print(vehicle_params)
        print(sim_params)

        g = sim_params.get("g",-9.81)
        erp = sim_params.get("erp",0.8)
        cfm = sim_params.get("cfm",1e-5)
        self.do3Dview = sim_params.get("do3Dview", True)
        self.substeps = sim_params.get("substeps", 2)
        gridsize = sim_params.get("gridsize", 20)

        vp = vehicle_params
        front_mass, beam_mass, wheel_mass = vp["front_mass"], vp["beam_mass"], vp["wheel_mass"]
        fx, fy, fz = vp["front_length"], vp["front_width"], vp["front_height"]
        bx, by, bz = vp["beam_length"], vp["beam_width"], vp["beam_height"]
        wheel_clearing = vp["wheel_clearing_z"]
        wr, ww = vp["wheel_radius"], vp["wheel_width"]

        self.mu1, self.mu2 = vp["mu1"], vp["mu2"]
        self.frictionLimit1 = self.mu1 * (front_mass + 2*beam_mass + 4*wheel_mass)/4 * abs(g)
        self.frictionLimit2 = self.mu2 * (front_mass + 2*beam_mass + 4*wheel_mass)/4 * abs(g)


        # setup ode
        self.world = ode.World()
        self.world.setGravity( (0,0,g) )
        self.world.setERP(erp)
        self.world.setCFM(cfm)
        self.space = ode.Space()
        self.ground = Plane( (0,0,1), 0, self.world, self.space)
        self.contactgroup = ode.JointGroup()

        # setup vehicle
        zw = 0.1 + wr
        zc = zw + wheel_clearing + fz/2
        self.front = Box(front_mass, (fx,fy,fz), (0,0,zc), (0,0,0), self.world, self.space)
        beam_xc = fx/2 - bx/2
        beam_yc = fy/2 -by/2
        beam_zc = zc + fz/2 + bz/2
        self.beam_l = Box(beam_mass, (bx,by,bz), (beam_xc,  beam_yc, beam_zc), (0,0,0), self.world, self.space)
        self.beam_r = Box(beam_mass, (bx,by,bz), (beam_xc, -beam_yc, beam_zc), (0,0,0), self.world, self.space)

        rpy = (np.pi/2,0,0)
        yw = fy/2 + wr
        xwr = beam_xc - bx/2 + wr
        self.wheel_fl = Cylinder(wheel_mass, wr, ww, (0,  yw, zw), rpy, self.world, self.space)
        self.wheel_fr = Cylinder(wheel_mass, wr, ww, (0, -yw, zw), rpy, self.world, self.space)
        self.wheel_rl = Cylinder(wheel_mass, wr, ww, (xwr,  yw, zw), rpy, self.world, self.space)
        self.wheel_rr = Cylinder(wheel_mass, wr, ww, (xwr, -yw, zw), rpy, self.world, self.space)

        # connect bodies
        self.joint_front_beam_l = connect_fixed(self.front, self.beam_l, self.world)
        self.joint_front_beam_r = connect_fixed(self.front, self.beam_r, self.world)
        self.joint_wheel_fl = connect_wheel_to_body(self.wheel_fl, self.front, self.world)
        self.joint_wheel_fr = connect_wheel_to_body(self.wheel_fr, self.front, self.world)
        self.joint_wheel_rl = connect_wheel_to_body(self.wheel_rl, self.beam_l, self.world)
        self.joint_wheel_rr = connect_wheel_to_body(self.wheel_rr, self.beam_r, self.world)
        # store wheel joints in order fl, rl, rr, fr
        self.joint_wheels = [
                self.joint_wheel_fl,
                self.joint_wheel_rl,
                self.joint_wheel_rr,
                self.joint_wheel_fr
        ]
        assert self.joint_wheel_fl is self.joint_wheels[0]

        # setup rendering
        if self.do3Dview:
            self.window = window.Window("3D view", 1000,800)
            self.renderer = RendererCollection(program.Program(shader.COLOR_SHADERS))
            self.planerenderer = HorizontalPlaneRenderer(gridsize, program.Program(shader.LINE_SHADERS))

            boxes = [
                    self.front,
                    self.beam_l,
                    self.beam_r,
            ]
            cylinders = [
                    self.wheel_fl,
                    self.wheel_rl,
                    self.wheel_rr,
                    self.wheel_fr,
            ]

            box_renderers = [BoxRenderer(b) for b in boxes]
            cylinder_renderers = [CylinderRenderer(c) for c in cylinders]
            self.renderer.add(*box_renderers, *cylinder_renderers)

        
    def _near_callback(self, args, geom1, geom2):
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
            c.setMu(self.frictionLimit1) 
            c.setMu2(self.frictionLimit2)
            j = ode.ContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())

    def setTorques(self, steer_torques, drive_torques):
        """
        set angular drives on fl, rl, rr, fr
        """
        for i in range(len(self.joint_wheels)):
            self.joint_wheels[i].addTorques(steer_torques[i], drive_torques[i])


    def step(self, t, dt):
        steps = self.sim_params.get("substeps", 2)

        shouldStop = False
        # step simulation with substepping
        for i in range(steps):
            self.space.collide(
                    (self.world, self.contactgroup),
                    self._near_callback)
            self.world.step(dt/self.substeps)
            self.contactgroup.empty()

        # update render
        if self.do3Dview:
            # setup drawing
            self.window.clear()

            # camera
            posx,posy,posz = self.front.position
            rc, zc = 10, 5
            cx = rc*np.cos(0.1*t)
            cy = rc*np.sin(0.1*t)
            eye = glm.vec3(posx+cx,posy+cy,posz+zc)
            target = glm.vec3(posx,posy,posz)
            view = glm.lookAt(eye, target, glm.vec3(0,0,1))

            proj = glm.perspective(glm.radians(80), self.window.width/self.window.height, 0.01, 1000.0)
            projview = proj*view

            self.renderer.draw(eye, projview)
            self.planerenderer.draw(eye, projview)

            self.window.swap()
            self.window.poll_events()

            if self.window.shouldClose():
                shouldStop = True

        return shouldStop



