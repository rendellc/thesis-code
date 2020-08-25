import numpy as np

class Vehicle:
    def __init__(self):
        self.distance_front_axle = 1.0
        self.distance_rear_axle = 2.0
        self.width_front = 1.0
        self.width_rear = 1.0

        self.pos_cog = np.zeros((3,))
        self.rpy_cog = np.zeros((3,))
        self.vel_cog = np.zeros((3,))

        print(self.pos_cog)
        print(self.vel_cog)



def main():
    vehicle = Vehicle()

    print(vehicle)


if __name__=="__main__":
    main()
