import glfw
import ode
import glm
import numpy as np


from box import Box
from cylinder import Cylinder
from wheel import Wheel
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

def center_of_mass(bodies):
    total_mass = 0
    mass_center = np.zeros(3)
    for b in bodies:
        pos = b.position
        mass = b.mass.mass
        mass_center += mass*pos
        total_mass += mass

    mass_center = mass_center/total_mass
    return mass_center




def connect_wheel_to_body(wheel, body, world):
    joint = ode.Hinge2Joint(world)
    joint.attach(body.body, wheel.inner.body)
    joint.setAnchor(wheel.position)
    joint.setAxis1((0,0,1)) # axis 1 is specified relative to body 1
    joint.setAxis2((0,1,0)) # axis 2 is specified relative to body 2
    return joint


class VehicleSim:
    def __init__(self, vehicle_params, sim_params):
        self.sim_params = sim_params
        # print(vehicle_params)
        # print(sim_params)

        g = sim_params.get("g",-9.81)
        erp = sim_params.get("erp",0.8)
        cfm = sim_params.get("cfm",1e-5)
        self.do3Dview = sim_params.get("do3Dview", True)
        self.substeps = sim_params.get("substeps", 2)
        gridsize = sim_params.get("gridsize", 20)
        self.friction_scale = sim_params.get("friction_scale", 1)

        body_color = np.array(sim_params.get("body_color", [0.91,0.96,0.95]))
        wheel_color = np.array(sim_params.get("wheel_color", [0.61,0.7,0.6]))
        tire_color = np.array(sim_params.get("tire_color", [0.1,0.1,0.1]))

        vp = vehicle_params
        front_mass = vp.get("front_mass", 0)
        beam_mass =  vp.get("beam_mass", 0)
        wheel_mass_inner = vp["wheel_mass_inner"]
        tire_mass = vp["wheel_mass_tire"]
        fx, fy, fz = vp["front_length"], vp["front_width"], vp["front_height"]
        bx, by, bz = vp["beam_length"], vp["beam_width"], vp["beam_height"]
        wheel_clearing = vp["wheel_clearing_z"]
        wr, ww = vp["wheel_radius"], vp["wheel_width"]
        front_track = vp["front_track"]
        rear_track = vp["rear_track"]

        wheel_mass = wheel_mass_inner + tire_mass

        if vp.get("use_body_mass", True):
            body_mass = vp["body_mass"]
            front_volume = fx*fy*fz
            beam_volume = bx*by*bz
            body_volume = front_volume + 2*beam_volume
            density = body_mass/body_volume
            front_mass = front_volume*density
            beam_mass = beam_volume*density

        assert front_mass != 0 and beam_mass != 0



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
        self.front = Box(front_mass, (fx,fy,fz), (0,0,zc), (0,0,0), self.world, self.space, doCollide=False)
        beam_xc = fx/2 - bx/2 - 0.1
        beam_yc = fy/2 
        beam_zc = zc + fz/2 
        self.beam_l = Box(beam_mass, (bx,by,bz), (beam_xc,  beam_yc, beam_zc), (0,0,0), self.world, self.space, doCollide=False)
        self.beam_r = Box(beam_mass, (bx,by,bz), (beam_xc, -beam_yc, beam_zc), (0,0,0), self.world, self.space, doCollide=False)

        rpy = (np.pi/2,0,0)
        #yw = fy/2 + wr
        yw_front = front_track/2
        yw_rear = rear_track/2
        xwr = beam_xc - bx/2 + wr
        self.wheel_fl = Wheel(wheel_mass_inner, 0.6*wr, tire_mass, wr,
                ww, (0,  yw_front, zw), rpy, self.world, self.space)
        self.wheel_fr = Wheel(wheel_mass_inner, 0.6*wr, tire_mass, wr,
                ww, (0, -yw_front, zw), rpy, self.world, self.space)
        self.wheel_rl = Wheel(wheel_mass_inner, 0.6*wr, tire_mass, wr,
                ww, (xwr, yw_rear, zw), rpy, self.world, self.space)
        self.wheel_rr = Wheel(wheel_mass_inner, 0.6*wr, tire_mass, wr,
                ww, (xwr, -yw_rear, zw), rpy, self.world, self.space)

        # connect bodies
        self.joint_front_beam_l = connect_fixed(self.front, self.beam_l, self.world)
        self.joint_front_beam_r = connect_fixed(self.front, self.beam_r, self.world)
        self.joint_wheel_fl = connect_wheel_to_body(self.wheel_fl, self.front, self.world)
        self.joint_wheel_fr = connect_wheel_to_body(self.wheel_fr, self.front, self.world)
        self.joint_wheel_rl = connect_wheel_to_body(self.wheel_rl, self.beam_l, self.world)
        self.joint_wheel_rr = connect_wheel_to_body(self.wheel_rr, self.beam_r, self.world)

        self.joint_tire_fl = connect_fixed(self.wheel_fl.inner, self.wheel_fl.tire, self.world)
        self.joint_tire_rl = connect_fixed(self.wheel_rl.inner, self.wheel_rl.tire, self.world)
        self.joint_tire_rr = connect_fixed(self.wheel_rr.inner, self.wheel_rr.tire, self.world)
        self.joint_tire_fr = connect_fixed(self.wheel_fr.inner, self.wheel_fr.tire, self.world)


        # store wheel joints in order fl, rl, rr, fr
        self.joint_wheels = [
                self.joint_wheel_fl,
                self.joint_wheel_rl,
                self.joint_wheel_rr,
                self.joint_wheel_fr
        ]
        assert self.joint_wheel_fl is self.joint_wheels[0]

        mass_center_in_world = center_of_mass([self.front, 
            self.beam_l, self.beam_r, 
            self.wheel_fl.inner, self.wheel_fl.tire,
            self.wheel_rl.inner, self.wheel_rl.tire,
            self.wheel_rr.inner, self.wheel_rr.tire,
            self.wheel_fr.inner, self.wheel_fr.tire])
        self.mass_center_in_front = np.array(self.front.body.getPosRelPoint(mass_center_in_world))
        #print(mass_center_in_world, self.mass_center_in_front)

        self.wheels = [
                self.wheel_fl,
                self.wheel_rl,
                self.wheel_rr,
                self.wheel_fr,
        ]


        # setup rendering
        if self.do3Dview:
            self.window = window.Window("3D view", sim_params.get("window_width", 600), sim_params.get("window_height", 600))
            self.renderer = RendererCollection(program.Program(shader.COLOR_SHADERS))
            self.planerenderer = HorizontalPlaneRenderer(gridsize, program.Program(shader.LINE_SHADERS))

            boxes = [
                    self.front,
                    self.beam_l,
                    self.beam_r,
            ]
            box_renderers = [BoxRenderer(b, color=body_color) for b in boxes]

            cylinder_renderers = []
            for w in self.wheels:
                cylinder_renderers.append(
                    CylinderRenderer(w.inner, color=wheel_color, closed=True)
                )
                cylinder_renderers.append(
                    CylinderRenderer(w.tire, color=tire_color, closed=True)
                )

            self.renderer.add(*box_renderers, *cylinder_renderers)
            self.angle_camera = np.pi
            self.radius_camera = 6
            self.height_camera = 3

    def getWheelPositions(self):
        """
        Get the position of each wheel relative to center of mass in body frame.
        """
        positions = np.zeros((len(self.wheels),3))
        for i, w in enumerate(self.wheels):
            position_rel_front = np.array(self.front.body.getPosRelPoint(w.position))
            positions[i] = position_rel_front - self.mass_center_in_front

        return positions

            

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
            c.setMu(self.frictionLimit1 * self.friction_scale) 
            c.setMu2(self.frictionLimit2 * self.friction_scale)
            j = ode.ContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())

    def setTorques(self, steer_torques, drive_torques):
        """
        set angular drives on fl, rl, rr, fr
        """
        for i in range(len(self.joint_wheels)):
            self.joint_wheels[i].addTorques(steer_torques[i], drive_torques[i])

    def getPosition(self):
        """
        Return position of center of mass.
        """
        return np.array(self.front.body.getRelPointPos(self.mass_center_in_front))

    def getVelocity(self):
        """
        Return velocity of center of mass in world frame.
        """
        return np.array(self.front.body.getRelPointVel(self.mass_center_in_front))

    def getRPY(self):
        return self.front.rpy

    def getRPYRate(self):
        return self.front.rpyrate

    def getWheelDriveRates(self):
        return -np.array([joint.getAngle2Rate() for joint in self.joint_wheels])

    def getWheelSteerAngles(self):
        return -np.array([joint.getAngle1() for joint in self.joint_wheels])

    def getWheelSteerRates(self):
        return -np.array([joint.getAngle1Rate() for joint in self.joint_wheels])

    def setFrictionScale(self, scale):
        self.friction_scale = scale

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
            step_angle_camera, step_radius_camera, step_height_camera = 0.01, 0.99, 0.1
            if glfw.get_key(self.window.window, glfw.KEY_A) == glfw.PRESS:
                self.angle_camera += step_angle_camera 
            if glfw.get_key(self.window.window, glfw.KEY_D) == glfw.PRESS:
                self.angle_camera -= step_angle_camera
            if glfw.get_key(self.window.window, glfw.KEY_Z) == glfw.PRESS:
                self.radius_camera *= step_radius_camera
            if glfw.get_key(self.window.window, glfw.KEY_X) == glfw.PRESS:
                self.radius_camera *= 1 + (1 - step_radius_camera)
            if glfw.get_key(self.window.window, glfw.KEY_S) == glfw.PRESS:
                self.height_camera -= step_height_camera
            if glfw.get_key(self.window.window, glfw.KEY_W) == glfw.PRESS:
                self.height_camera += step_height_camera

            posx,posy,posz = self.getPosition()
            _,_,yaw = self.getRPY()
            cx = self.radius_camera*np.cos(yaw+self.angle_camera + 0.0*t)
            cy = self.radius_camera*np.sin(yaw+self.angle_camera + 0.0*t)
            cz = self.height_camera
            eye = glm.vec3(posx+cx,posy+cy,posz+cz)
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



