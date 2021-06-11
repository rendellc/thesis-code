import numpy as np

from report_utils.plan_parser import plan_file_to_cartesian

from copy import deepcopy

WP_SINGLE_TURN = np.array([[0, 0],
                           [30, 0],
                           [30, 15],
                           ],
                          dtype=float)


WP_SIMPLE_LAP = np.array([[0, 0],
                          [15, 0],
                          [30, 30],
                          [10, 30],
                          [-20, 45],
                          [-10, 0],
                          [0, 0],
                          ],
                         dtype=float)

WP_SURVEY = plan_file_to_cartesian(
    "/home/cale/thesis-code/ws/launch/survey_dragvoll_manual.plan")

VEHICLE_CONTROLLER_PARAMETERS = [
    {"update_rate": 50.0},
    {"pid_active": True},
    {"yaw_source": "path"},
    {"yawrate_sat": 1.0},
    {"P_yaw": 10.0},
    {"I_yaw": 0.0},
    {"D_yaw": 5.0},
    {"yawrate_ff": 1.0},
    {"P_yawrate": 0.0},
    {"I_yawrate": 0.5},
    {"D_yawrate": 0.0},
    {"P_speed": 1.0},
    {"I_speed": 0.1},
    {"D_speed": 0.0},
    {"ilqr_4wis_active": False},
    {"ilqr_cost_x": 1.0},
    {"ilqr_cost_y": 1.0},
    {"ilqr_cost_yaw": 10.0},
    {"ilqr_cost_angular_vel": 0.2},
    {"ilqr_cost_steering_angle": 1.0},
    {"ilqr_trajectory_length": 50},
    {"ilqr_singletrack_active": False},
    {"disable_all": False}
]

WHEEL_CONTROLLER_PARAMETERS = [
    {"update_rate": 100.0},
    {"P_omega": 250.0},
    #{"P_omega": 100.0},
    {"I_omega": 0.0},
    {"D_omega": 0.0},
    {"P_delta": 5.0},
    {"use_robust_rate": False},
    {"robust_rate_softregion": 5.0},
    {"steering_rate_limit": np.deg2rad(60.0)},
    {"max_steering_accel": 0.1},
    {"wheel_mass": 200.0},
    {"wheel_radius": 0.505},
    {"wheel_width": 0.4},
    {"use_sliding_mode": True},
    {"sliding_mode_softregion": 5.0},
    {"sliding_mode_eigenvalue": 5.0},
    {"steer_resistance_factor": 2.0},
    {"beta_0": 0.1},
    {"use_reference_optimization": True},
    {"use_reference_model": True},
    {"steer_damping_ratio": 0.9},
    {"steer_natural_frequency": 3.0},
    {"drive_time_constant": 1.0}
]


GUIDANCE_PARAMETERS = [
    {"update_rate": 50.0},
    {"use_fermat_smoothing": True},
    {"use_circular_smoothing": False},
    {"use_braking": False},
    {"maximum_curvature": 0.75},
    {"approach_angle": np.deg2rad(30)},
    # {"P_approach": 2.0}  # NOTE: mclain recommends 1/radius_min = maximum_curvature
    # {"P_approach": 2.0},
    {"P_approach": 2.0},
    {"I_approach": 0.0},
    {"D_approach": 0.0},
    {"speed_desired": 8.0/3.6}
]


def update_parameter(parameter_list, name, value):
    updated_parameter_list = deepcopy(parameter_list)

    new_param = {name: value}
    found = False
    for i, param in enumerate(updated_parameter_list):
        pname = next(iter(param.keys()))
        if pname == name:
            updated_parameter_list[i] = new_param
            found = True
            break
    if not found:
        updated_parameter_list.append(new_param)

    return updated_parameter_list


# waypoints = np.array([
#     [0, 0],
#     [30, 0],
#     [30, 30]
#     #[20, 20],
#     #[0, 25],
#     #[-10, 10],
#     #[-10, 0],
#     #[0, 0]
# ], dtype=float)
