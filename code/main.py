import vehicle

import numpy as np

import simulate as sim


def main():
    config = {
            "mass": 2.5e3,
            "width_front": 2.0,
            "width_rear": 2.0,
            "length_front": 1.0,
            "length_rear": 2.5,
            "height": 2.5,
            "wheel_mass": 300.0,
            "wheel_radius": 1.0,
            "wheel_width": 0.4,
            "cog_start_position": np.array([0.,0.,0.]),
            "cog_start_velocity": np.array([0.1,0.,0.]),
            "cog_start_rpy": np.array([0.,0.,0.]),
            "cog_start_rpy_rate": np.array([0.,0.,0.]),
            }
    car = vehicle.Car2D(config)

    run_loop = True
    while run_loop:
        try:
            # Update controller


            # Physics computations


            # Step simulation forward
            sim.simulate_step_forward(car, 1e-2)

            print(car)
        except KeyboardInterrupt:
            run_loop = False










if __name__=="__main__":
    main()
