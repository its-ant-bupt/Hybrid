import numpy as np
from lib.Grid_A_Star import GridAStar
from lib.HybridAStar import HybridAStar, Path
from lib.GetObs import GetObs
from lib.Vehicle import Vehicle
from lib.util import *
from lib.VehicleAnimation import show
from settings import args
import time
import math
import csv

path_num = args.path_num
inter = args.inter
ObstList, ObstLine, Start, End, ObsNum, ObstLineNum, ObstLineList, case = GetObs(path_num, inter=inter)
ObstList = np.array(ObstList)
ObstLine = np.array(ObstLine)

# temp = Start
# Start = End
# End = temp

Vehicle = Vehicle()

# Setting the config
params = \
    {
        # The config of obs
        "ObstList": ObstList,
        "ObstLine": ObstLine,
        "ObsNum": ObsNum,
        "ObsLineNum": ObstLineNum,
        "ObsLineList": ObstLineList,

        # The config of motion resolution
        "MOTION_RESOLUTION": args.motion_resolution,
        "N_STEER": args.n_steer,
        "EXTEND_AREA": args.extend_area,
        "XY_GRID_RESOLUTION": args.xy_grid_resolution,
        "YAW_GRID_RESOLUTION": deg2rad(args.yaw_grid_resolution),

        # Grid bound
        "MINX": min(ObstList[:, 0]) - args.extend_area,
        "MAXX": max(ObstList[:, 0]) + args.extend_area,
        "MINY": min(ObstList[:, 1]) - args.extend_area,
        "MAXY": max(ObstList[:, 1]) + args.extend_area,
        "MINYAW": args.min_yaw,
        "MAXYAW": args.max_yaw,

        # cost related define
        "SB_COST": args.sb_cost,
        "BACK_COST": args.back_cost,
        "STEER_CHANGE_COST": args.steer_change_cost,
        "STEER_COST": args.steer_cost,
        "H_COST": args.h_cost,

        # added item
        "MUST_SHORTEST": args.must_shortest,
        "LEFT_FIRST": not args.right_first,  # 方向盘先向左打遍历
        "GO_FIRST": not args.back_first  # 先向正方向遍历
    }
print(params["MINY"])
print(params["MINX"])
params["XWIDTH"] = math.ceil((params["MAXX"] - params["MINX"]) / params["XY_GRID_RESOLUTION"])
params["YWIDTH"] = math.ceil((params["MAXY"] - params["MINY"]) / params["XY_GRID_RESOLUTION"])

Anchor = [-6.5, -6, -math.pi/2]  # case 19

# ==============================查找路径，判断是否有锚点，若有则进行二次寻路===============
params["Anchoring"] = False
if args.anchor:
    params["Anchoring"] = True
    params["LEFT_FIRST"] = False
    params["GO_FIRST"] = False
    # params["MOTION_RESOLUTION"] = 0.25
    # params["N_STEER"] = 25
    # params["XY_GRID_RESOLUTION"] = 0.5
    # params["YAW_GRID_RESOLUTION"] = deg2rad(30)
    time1 = time.time()
    GAS = GridAStar(ObstList, [Anchor[0], Anchor[1]], params["XY_GRID_RESOLUTION"], params)
    ObstMapAnchor = GAS.FindCostMap()
    time2 = time.time()
    print("The anchor run time of AStar is {}".format(time2 - time1))
    params["ObstMap"] = ObstMapAnchor

    time1 = time.time()
    HAS = HybridAStar(Start=Start, End=Anchor, Vehicle=Vehicle, Config=params)
    pathAdd = HAS.findPath()
    time2 = time.time()
    print("The anchor run time of Hybrid AStar is {}".format(time2 - time1))

    Start = [pathAdd.x[-1], pathAdd.y[-1], pathAdd.yaw[-1]]
    params["Anchoring"] = False
    params["MUST_SHORTEST"] = True

# End = [6.5, 3.5, End[2]]  # case 1 final
# End = [6.928907615, 3.773291449, 2.716767045] # case1 middle point
# End = [-6.5, -6, -math.pi/2]  # case 6 middle

time1 = time.time()
GAS = GridAStar(ObstList, [End[0], End[1]], params["XY_GRID_RESOLUTION"], params)
ObstMap = GAS.FindCostMap()
time2 = time.time()

print("The run time of AStar is {}".format(time2-time1))

params["ObstMap"] = ObstMap

# Start = [12.27208813, 3.750091997, 2.951194629]  # case 1 middle
# Start = [-6.5, -6, -math.pi/2]
# Start = [-6.5, -3.62056, -math.pi/2]

time1 = time.time()
HAS = HybridAStar(Start=Start, End=End, Vehicle=Vehicle, Config=params)
path = HAS.findPath()
time2 = time.time()
print("The run time of Hybrid AStar is {}".format(time2-time1))

if args.anchor:
    total_x = list(pathAdd.x) + list(path.x)
    total_y = list(pathAdd.y) + list(path.y)
    total_yaw = list(pathAdd.yaw) + list(path.yaw)
    total_direction = list(pathAdd.direction) + list(path.direction)
    total_cost = pathAdd.cost + path.cost
    path = Path(total_x, total_y, total_yaw, total_direction, total_cost)

# =============================计算误差，并保存文件====================================

StartDisError = math.hypot((path.x[0] - Start[0]), (path.y[0] - Start[1]))
StartYawError = max(abs(math.sin(path.yaw[0]) - math.sin(Start[2])),
                    abs(math.cos(path.yaw[0]) - math.cos(Start[2])))

EndDisError = math.hypot((path.x[-1] - End[0]), (path.y[-1] - End[1]))
EndYawError = max(abs(math.sin(path.yaw[-1]) - math.sin(End[2])),
                  abs(math.cos(path.yaw[-1]) - math.cos(End[2])))

print("Start bias dis: {}, yaw: {}...".format(StartDisError, StartYawError))
print("End bias dis: {}, yaw: {}...".format(EndDisError, EndYawError))

writePath = "./Result/traj-withoutTime{}.csv".format(path_num)
with open(writePath, 'w', newline="") as wf:
    wf_write = csv.writer(wf)
    X = path.x
    Y = path.y
    Yaw = path.yaw
    direction = path.direction
    wf_write.writerow([X[0], Y[0], Yaw[0], 0])
    for i in range(1, len(X)):
        if direction[i]:
            dis = math.hypot((X[i]-X[i-1]), (Y[i]-Y[i-1]))
        else:
            dis = -math.hypot((X[i] - X[i - 1]), (Y[i] - Y[i - 1]))
        wf_write.writerow([X[i], Y[i], Yaw[i], dis])

show(path, case, path_num)




