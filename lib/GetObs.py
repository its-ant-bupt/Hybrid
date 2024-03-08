import numpy as np
from lib.Case import Case
import math
x_bias = 0
y_bias = 0

def GetObs(path_num, inter=0.5):
    ObstList = []
    ObstLine = []
    case = Case.read('/home/zhengy/Code/HALOES/BenchmarkCases/Case%d.csv' % path_num)
    Start = [case.x0+x_bias, case.y0+y_bias, case.theta0]
    End = [case.xf+x_bias, case.yf+y_bias, case.thetaf]
    ObsNum = case.obs_num
    ObstLineNum = case.num_vertexes
    ObstLineList = case.obsLineList

    for obs_i in range(ObsNum):
        oriLine = list(case.obs[obs_i])
        oriLine.append(oriLine[0])
        oriLine = np.array(oriLine)

        for line_i in range(ObstLineNum[obs_i]):
            tLine = [oriLine[line_i, 0]+x_bias, oriLine[line_i, 1]+y_bias, oriLine[line_i+1, 0]+x_bias, oriLine[line_i+1, 1]+y_bias]
            ObstLine.append(tLine)
            ObstList.append([oriLine[line_i, 0]+x_bias, oriLine[line_i, 1]+y_bias])
            interNum = math.floor(math.hypot((oriLine[line_i+1, 0]-oriLine[line_i, 0]),
                                             (oriLine[line_i+1, 1]-oriLine[line_i, 1])) / inter)
            cosTheta = (oriLine[line_i+1, 0]-oriLine[line_i, 0]) / \
                       math.hypot((oriLine[line_i+1, 0]-oriLine[line_i, 0]), (oriLine[line_i+1, 1]-oriLine[line_i, 1]))
            sinTheta = (oriLine[line_i+1, 1]-oriLine[line_i, 1]) / \
                       math.hypot((oriLine[line_i+1, 0]-oriLine[line_i, 0]), (oriLine[line_i+1, 1]-oriLine[line_i, 1]))
            for inter_i in range(interNum):
                ObstList.append([oriLine[line_i, 0]+x_bias + cosTheta*(inter_i+1)*inter,
                                 oriLine[line_i, 1]+y_bias + sinTheta*(inter_i+1)*inter])

    return ObstList, ObstLine, Start, End, ObsNum, ObstLineNum, ObstLineList, case


if __name__ == '__main__':
    ObstList, ObstLine, Start, End, ObsNum, ObstLineNum, ObstLineList, case = GetObs(1)
    print("------")
