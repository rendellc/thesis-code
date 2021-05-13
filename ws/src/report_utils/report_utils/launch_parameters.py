import numpy as np

from report_utils.plan_parser import plan_file_to_cartesian

WP_SINGLE_TURN = np.array([[0, 0],
                           [30, 0],
                           [30, 30],
                           ],
                          dtype=float)


WP_SIMPLE_LAP = np.array([[0, 0],
                          [15, 0],
                          [30, 30],
                          [10, 30],
                          [10, 40],
                          [-10, 45],
                          [-10, 0],
                          [0, 0],
                          ],
                         dtype=float)

WP_SURVEY = plan_file_to_cartesian(
    "/home/cale/thesis-code/ws/launch/survey_dragvoll_manual.plan")

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
