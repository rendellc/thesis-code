import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrow, Rectangle, Polygon



# Size of vehicle
Lf = 1.5 # distance to front axle cog
Lr = 1 # distance to rear axle from cog
Bf = 2 # distance between front wheels

lower_left = [-Lr, -Bf/2]
chassis = Rectangle(lower_left, Lr+Lf, Bf, fill=False, )


cog_vel_arrow = FancyArrow(0,0,1,1)
wheel_fr_arrow = FancyArrow(0,0,1,1)


fig, ax = plt.subplots()
ax.grid(True)
xdata, ydata = [], []

def init():
    print("init")

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.add_patch(chassis)
    ax.add_patch(cog_vel_arrow)
    ax.add_patch(wheel_fr_arrow)

    return chassis, cog_vel_arrow, wheel_fr_arrow



def update(frame):
    position_cog, velocity_cog, yaw, yaw_rate = frame

    ts = ax.transData
    chassis_translate = mpl.transforms.Affine2D().translate(position_cog[0], position_cog[1])

    origin_cog = ts.transform(position_cog)
    chassis_rotate = mpl.transforms.Affine2D().rotate_deg_around(origin_cog[0], origin_cog[1] ,yaw)
    t = chassis_translate + ts + chassis_rotate
    chassis.set_transform(t)

    
    global cog_vel_arrow
    ax.patches.remove(cog_vel_arrow) 
    cog_vel_arrow = plt.Arrow(position_cog[0], position_cog[1], 5*velocity_cog[0], 5*velocity_cog[1])
    ax.add_patch(cog_vel_arrow)

    global wheel_fr_arrow
    ax.patches.remove(wheel_fr_arrow) 
    distance_to_wheel = (Lf**2 + (Bf/2)**2)**0.5

    rotation_in_from_cog = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]])
    rotation_dot = np.array([
        [-np.sin(yaw), -np.cos(yaw)],
        [np.cos(yaw), -np.sin(yaw)]]) * yaw_rate
    position_wheel_fr_from_cog = np.array([Lf, -Bf/2])
    position_wheel_fr_from_in =  rotation_in_from_cog.dot(position_wheel_fr_from_cog)


    velocity_wheel_fr = velocity_cog + rotation_dot.dot(position_wheel_fr_from_cog )
    wheel_fr_arrow = plt.Arrow(position_wheel_fr_from_in[0], position_wheel_fr_from_in[1],
            velocity_wheel_fr[0], velocity_wheel_fr[1])
    ax.add_patch(wheel_fr_arrow)


    return chassis, cog_vel_arrow, wheel_fr_arrow




def frame_generator():
    position_cog, velocity_cog = np.zeros((2,)), np.zeros((2,))
    yaw, yaw_rate = 0, 0 # deg, deg/sec

    i = 0

    while True:
        yield position_cog, velocity_cog, yaw, yaw_rate

        yaw_accel = 0 #0.1*np.random.randn()
        yaw_rate = 0.1 # yaw_accel
        yaw += yaw_rate

        velocity_cog[1] = 0.0*np.cos(0.02*i)
        position_cog += velocity_cog
        i += 1

        print(yaw)

ani = animation.FuncAnimation(fig, update, frames=frame_generator(), init_func=init, interval=30)

plt.show()
