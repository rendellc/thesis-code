
import pygame
from pygame.locals import *
import ode
import random

scale = 40
def coord(x,y):
    "Convert world coordinates to pixel coordinates"
    return int(320+scale*x), int(240-scale*y)


def near_callback(args, geom1, geom2):
    # print("near_callback")
    # check if objects collid
    contacts = ode.collide(geom1, geom2)
    world, contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(1)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

def draw_box(srf, color, box):
    x,y,z = box.getPosition()
    lengths = box.getLengths()

    left, right = x - lengths[0]/2, x + lengths[0]/2
    bottom, top = y - lengths[1]/2, y + lengths[1]/2
    left, bottom = coord(left,bottom)
    right, top = coord(right,top)

    rect = pygame.Rect(left, bottom, right-left, top-bottom)
    pygame.draw.rect(srf, color, rect)

def draw_ball(srf, color, ball):
    x,y,z = ball.getPosition()

    pygame.draw.circle(srf, color, coord(x,y), scale*ball.radius, 0)

# open display
pygame.init()
srf = pygame.display.set_mode((640,480))

# setup world
world = ode.World()
world.setGravity((0,-9.81,0))
world.setERP(0.8)
world.setCFM(1e-5)

space = ode.Space()

ground = ode.GeomPlane(space, (0,1,0), -2.5)
aabb = ground.getAABB()
normal, dist = ground.getParams()

box = ode.GeomBox(space, lengths=(1,1,1))
box.setPosition((5,dist+0.5,0))
box.shape = "box"


# create bodies
def create_sphere(world, space, density, radius):
    body = ode.Body(world)
    M = ode.Mass()
    M.setSphere(density, radius)
    body.shape = "sphere"
    body.radius = radius
    body.setMass(M)

    geom = ode.GeomSphere(space=space, radius=radius)
    geom.setBody(body)

    return body, geom

N = 10
objects = [create_sphere(world, space, 2500, 1) for n in range(N)]
balls = [obj[0] for obj in objects]
ballgeoms = [obj[1] for obj in objects]

for b in balls:
    b.setPosition((random.gauss(0,5), random.gauss(2,5), 0))


print("num geoms", space.getNumGeoms())

# # connect b1 with the static environment
# j1 = ode.BallJoint(world)
# j1.attach(b1, ode.environment)
# j1.setAnchor((0,0,0))
# #j1.setAxis((0,0,1))
# 
# # connect b2 with b1
# j2 = ode.BallJoint(world)
# j2.attach(b1, b2)
# j2.setAnchor(b1.getPosition())

contactgroup = ode.JointGroup()

# simulation loop
fps = 50
dt = 1/fps
loopFlag = True
clk = pygame.time.Clock()


while loopFlag:
    events = pygame.event.get()
    for e in events:
        if e.type == QUIT:
            loopFlag = False
        if e.type == KEYDOWN:
            loopFlag = False

    # clear screen
    srf.fill((255,255,255))

    # motor input
    #j1.addTorque(40)
    #j1.setParam(ode.ParamFMax, 100)


    # draw the bodies
    draw_box(srf, (100,50,80), box)
    for i, b in enumerate(balls):
        x,y,z = b.getPosition()

        if y > -10 or y > 10:
            draw_ball(srf, (55,0,200), b)
        else:
            print("removing", i)
            space.remove(ballgeoms[i])

    # x1,y1,z1 = b1.getPosition()
    # x2,y2,z2 = b2.getPosition()
    # pygame.draw.circle(srf, (55,0,200), coord(x1,y1), b1.radius, 0)
    # pygame.draw.line(srf, (55,0,200), coord(0,0), coord(x1,y1), 2)
    # pygame.draw.circle(srf, (100,0,150), coord(x2,y2), b2.radius, 0)
    # pygame.draw.line(srf, (55,0,200), coord(x1,y1), coord(x2,y2), 2)

    # draw ground
    pygame.draw.line(srf, (55,55,55), coord(-10,dist), coord(10,dist), 2)

    pygame.display.flip()

    # step simulation
    substeps = 2
    for i in range(substeps):
        space.collide((world, contactgroup), near_callback)
        world.step(dt/substeps)
        contactgroup.empty()

    # rate limit
    clk.tick(fps)



