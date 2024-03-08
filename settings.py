import argparse
import numpy as np

parser = argparse.ArgumentParser(description='TPCAP Parameters')


parser.add_argument("--path_num", type=int, default=24, help="The path number of cases")
parser.add_argument("--inter", type=float, default=0.5, help="The inter dis of obs")

parser.add_argument("--motion_resolution", type=float, default=0.1, help="The motion resolution of vehicle")
parser.add_argument("--n_steer", type=int, default=25, help="The number of steer command")
parser.add_argument("--extend_area", type=float, default=10, help="The map extend length")
parser.add_argument("--xy_grid_resolution", type=float, default=0.5, help="The resolution of grid map")
parser.add_argument("--yaw_grid_resolution", type=float, default=15, help="The resolution of deg [deg]")
parser.add_argument("--min_yaw", type=float, default=-np.pi, help="The min yaw")
parser.add_argument("--max_yaw", type=float, default=np.pi, help="The max yaw")

parser.add_argument("--sb_cost", type=float, default=10, help="The cost of switch back")
parser.add_argument("--back_cost", type=float, default=0, help="The cost of run back")
parser.add_argument("--steer_change_cost", type=float, default=1.5, help="The cost of steer change")
parser.add_argument("--steer_cost", type=float, default=1.5, help="The cost of steer")
parser.add_argument("--h_cost", type=float, default=10, help="The cost of heuristic")

parser.add_argument("--anchor", action="store_true", default=False, help="Using the middle anchor")
parser.add_argument("--must_shortest", action="store_true", default=False, help="Whether search the shortest path "
                                                                                "if not we will find"
                                                                                " the reachable path firstly~")
parser.add_argument("--right_first", action="store_true", default=False, help="First check the right")
parser.add_argument("--back_first", action="store_true", default=False, help="First go back")


parser.add_argument("--test_batch", type=int, default=10, help="Run batch to test the avg run time.")

args = parser.parse_args()