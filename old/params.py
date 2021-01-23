"""
Lengde: 4110 mm
Lengde mellom akslinger 2830 mm.
Hjuldiameter er 1010 mm.
Det gjør at fronten er 270 mm foran framhjulene eller 775 mm foran framaksel.

Bredde er på 2400 mm i ytterkant av hjulene og det er 2000 mm mellom senter av hjulene så da tolker jeg at hjulene er 400 mm brede.

Vekt er 2380 kg uten nyttelast. Opp mot 2000 kg ekstra med nyttelast. Maks vekt altså ~4400 kg
"""
axle_to_axle = 2.830
wheel_diameter = 1.010
total_length = 4.110
front_axle_to_front = 0.775
wheel_width = 0.400
total_width = 2.400

wheel_radius = wheel_diameter/2

# vehicle parameters
VEHICLE_PARAMS = dict(
        body_mass=1580,
        wheel_mass_inner=100,
        wheel_mass_tire=100,
        front_length=1,
        front_width=total_width - wheel_width - wheel_radius - 0.3,
        front_height=1,
        front_track=total_width - wheel_width, # distance between front wheel centers
        rear_track=total_width - wheel_width, # distance between rear wheel centers
        beam_length=4.110,
        beam_width=0.2,
        beam_height=0.3,
        wheel_clearing_z=0.1,
        wheel_radius=1.010/2,
        wheel_width=0.4,
        mu1=0.8,
        mu2=0.3,
)


# simulator parameters
SIM_PARAMS = dict(
        body_color=[0.7,0.7,0.7],
        do3Dview=True,
        record3Dfilename="",
        record3Dfps=30,
        substeps=4,
        window_width=1600,
        window_height=800,
        camera_start_angle=0.75*3.14,
        camera_auto_yaw=0.01,
        friction_scale=1,
)
