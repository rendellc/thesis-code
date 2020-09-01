import vehicle



def main():
    config = {
            "mass": 2.5e3,
            "width_front": 2.0,
            "width_rear": 2.0,
            "length_front": 1.0,
            "length_rear": 2.5,
            "wheel_radius": 1.0,
            "wheel_width": 0.4,
            }
    car = vehicle.Car2D(config)



if __name__=="__main__":
    main()
