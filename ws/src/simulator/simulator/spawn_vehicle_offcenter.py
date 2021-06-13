from simulator.spawn_vehicle import spawn
from numpy import pi


def main(args=None):
    spawn((20.0, 20.0, 1.15975), pi)
    #spawn((1.0, 0.0, 1.598), 0.0)


if __name__ == "__main__":
    main()
