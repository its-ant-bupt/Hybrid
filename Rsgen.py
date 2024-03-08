import numpy as np
from lib.HybridAStar import HybridAStar, Path
from lib.GetObs import GetObs
from lib.Vehicle import Vehicle
from lib.util import *
from lib.VehicleAnimation import show
from lib.CheckCollision import CheckCollision
from settings import args
from lib.RSPath import *
import time
import math
import csv

class Path(object):

    def __init__(self, lengths, ctypes, L, x, y, yaw, directions):
        self.lengths = lengths
        self.ctypes = ctypes
        self.L = L
        self.x = x
        self.y = y
        self.yaw = yaw
        self.directions = directions

path_num = args.path_num
inter = args.inter
ObstList, ObstLine, Start, End, ObsNum, ObstLineNum, ObstLineList, case = GetObs(path_num, inter=inter)
ObstList = np.array(ObstList)
ObstLine = np.array(ObstLine)

maxc = math.tan(0.75) / 2.8
path = Path
# path.lengths = [22*maxc, -1*maxc, -1*maxc, -1.5*maxc, 2*maxc, 1.6*maxc, -1.7*maxc, 3.7*maxc]  # case 6
# path.ctypes = ["S", "R", "S", "R", "S", "L", "R", "L"]
# path.lengths = [18.5*maxc, 4.5*maxc, 1*maxc, -0.5*maxc, -1.55*maxc, -2*maxc, 2*maxc]  # case 6
# path.ctypes = ["S", "L", "S", "R", "S", "R", "L"]
# path.lengths = [4*maxc, 21*maxc, -5*maxc, -10*maxc]  # case 7
# path.ctypes = ["L", "S", "R", "S"]
# path.lengths = [2*maxc, 22*maxc, 3*maxc, 4*maxc, -3.5*maxc, -3*maxc]  # case 7
# path.ctypes = ["L", "S", "L", "S", "R", "S"]

# path.lengths = [0.75*maxc, -3.5*maxc, -5*maxc]  # case 2
# path.ctypes = ["R", "S", "R"]

path.lengths = [0.5*maxc, 1*maxc, 5.5*maxc]  # case 5
path.ctypes = ["L", "S", "R"]



path.L = sum([abs(i) for i in path.lengths])
step_size = 0.1
q0 = Start



x, y, yaw, directions = generate_local_course(path.L, path.lengths, path.ctypes, maxc, step_size * maxc)
# convert global coordinate
path.x = [math.cos(-q0[2]) * ix + math.sin(-q0[2])* iy + q0[0] for (ix, iy) in zip(x, y)]
path.y = [-math.sin(-q0[2]) * ix + math.cos(-q0[2])* iy + q0[1] for (ix, iy) in zip(x, y)]
path.yaw = [pi_2_pi(iyaw + q0[2]) for iyaw in yaw]
path.directions = directions
path.lengths = [l / maxc for l in path.lengths]
path.L = path.L / maxc

# final_paths = calc_paths(path.x[-1], path.y[-1], path.yaw[-1], End[0], End[1], End[2], maxc, step_size=step_size)
# final_path_set = {}
# veh = Vehicle
# path_id = 0
# for _path in final_paths:
#     final_path_set[path_id] = _path
#     path_id = path_id + 1
#
#
# p_id = min(
#     final_path_set, key=lambda o: final_path_set[o].L
# )
# final_path = final_path_set[p_id]
#
# total_x = list(path.x) + list(final_path.x)
# total_y = list(path.y) + list(final_path.y)
# total_yaw = list(path.yaw) + list(final_path.yaw)
# total_direction = list(path.directions) + list(final_path.directions)
# path = Path(None, None, None, total_x, total_y, total_yaw, total_direction)


writePath = "./Result/traj-withoutTime-RSgen{}.csv".format(path_num)
with open(writePath, 'w', newline="") as wf:
    wf_write = csv.writer(wf)
    X = path.x
    Y = path.y
    Yaw = path.yaw
    direction = path.directions
    wf_write.writerow([X[0], Y[0], Yaw[0], 0])
    for i in range(1, len(X)):
        if direction[i] == 1:
            dis = math.hypot((X[i]-X[i-1]), (Y[i]-Y[i-1]))
        else:
            dis = -math.hypot((X[i] - X[i - 1]), (Y[i] - Y[i - 1]))
        wf_write.writerow([X[i], Y[i], Yaw[i], dis])

show(path, case, 6)

