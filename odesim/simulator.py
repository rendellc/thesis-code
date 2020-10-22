import pygame
from pygame.locals import *
import ode

def coord(x,y, scale=80):
    "Convert world coordinates to pixel coordinates"
    return int(320+scale*x), int(240-scale*y)


# open display
pygame.init()
srf = pygame.display.set_mode((640,480))

# setup world
world = ode.World()
world.setGravity((0,-9.81,0))

space = ode.Space()
ground = ode.GeomPlane(space, (0,1,0), 0)

# create bodies
b1 = ode.Body(world)
M = ode.Mass()
M.setSphere(2500.0, 0.05)
M.mass = 1.0
b1.setMass(M)
b1.setPosition((1,2,0))

b2 = ode.Body(world)
M.setSphere(2500.0, 0.05)
b2.setMass(M)
b2.setPosition((2,2,0))

# connect b1 with the static environment
j1 = ode.HingeJoint(world)
j1.attach(b1, ode.environment)
j1.setAnchor((0,0,0))
j1.setAxis((0,0,1))

# connect b2 with b1
j2 = ode.BallJoint(world)
j2.attach(b1, b2)
j2.setAnchor((1,2,0))


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
    j1.addTorque(40)
    #j1.setParam(ode.ParamFMax, 100)

    # draw the bodies
    x1,y1,z1 = b1.getPosition()
    x2,y2,z2 = b2.getPosition()
    pygame.draw.circle(srf, (55,0,200), coord(x1,y1), 20, 0)
    pygame.draw.line(srf, (55,0,200), coord(0,0), coord(x1,y1), 2)
    pygame.draw.circle(srf, (100,0,150), coord(x2,y2), 20, 0)
    pygame.draw.line(srf, (55,0,200), coord(x1,y1), coord(x2,y2), 2)

    pygame.display.flip()

    # step simulation
    world.step(dt)

    # rate limit
    clk.tick(fps)


















