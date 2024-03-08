import numpy as np
import math
from lib.GetObs import GetObs
import cv2

class Node:
    def __init__(self, x, y, cost, p_ind):
        self.x = x
        self.y = y
        self.cost = cost  # 当前损失
        self.p_ind = p_ind  # 父亲节点

    def __str__(self):
        return str(self.x) + "," + str(self.y) + str(self.cost) + ',' + self.p_ind


class GridAStar:
    def __init__(self, obstList, goal, gres, params=None):
        self.obstList = np.array(obstList)
        self.goal = goal
        self.gres = gres
        self.obmap = []
        self.minx = -np.inf
        self.maxx = np.inf
        self.miny = -np.inf
        self.maxy = np.inf
        self.params = params

    def FindCostMap(self):
        minx, maxx, miny, maxy, obmap = self.CalcObstMap()
        # self.show(obmap)
        self.obmap = obmap
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        row = self.goal[0]
        col = self.goal[1]
        row = math.floor((row - minx) / self.gres)
        col = math.floor((col - miny) / self.gres)
        goal = [row, col]
        # costMap = obmap * 0

        pmap = self.AStarSearch(goal, obmap)

        return pmap

    def AStarSearch(self, goal, obmap):
        xwidth, ywidth = obmap.shape
        OpenList = {}
        CloseList = {}

        ngoal = Node(goal[0], goal[1], 0, "-1,-1")
        OpenList[self.getIndex(ngoal)] = ngoal

        iterNum = 0

        while len(OpenList) > 0:
            iterNum += 1
            c_id = self.PopOpen(OpenList)
            current = OpenList[c_id]

            # 从Open列表中删除当前点 并将其添加到Close中
            del OpenList[c_id]
            CloseList[c_id] = current

            # 遍历四周
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    adj_node_x = current.x + i
                    adj_node_y = current.y + j
                    if i == 0 and j == 0:
                        # 去掉自己的点
                        continue
                    elif adj_node_x < 0 or adj_node_x >= xwidth:
                        # 去掉边界外的点
                        continue
                    elif adj_node_y < 0 or adj_node_y >= ywidth:
                        continue
                    elif obmap[adj_node_x, adj_node_y] == 1:
                        # 去掉障碍物的点
                        continue

                    # 计算扩展点的G值，即从起点开始的路径损失
                    adj_cost = current.cost + math.hypot(i, j)

                    adj_node = Node(adj_node_x, adj_node_y, adj_cost, c_id)

                    adj_idx = self.getIndex(adj_node)

                    # 如果遍历到的点已经在close中了，继续执行
                    if adj_idx in CloseList.keys():
                        continue

                    if adj_idx in OpenList.keys():
                        if OpenList[adj_idx].cost > adj_cost:
                            OpenList[adj_idx].cost = adj_cost
                            OpenList[adj_idx].p_ind = c_id
                    else:
                        OpenList[adj_idx] = adj_node

        pmap = self.CalcPolicyMap(CloseList, xwidth, ywidth)
        print("iternum:", iterNum)
        return pmap

    @staticmethod
    def getIndex(node):
        return str(node.x) + "," + str(node.y)

    @staticmethod
    def PopOpen(OpenList):
        mincost = np.inf
        minidx = ""
        for key in OpenList.keys():
            cnode = OpenList[key]
            cost = cnode.cost
            if cost < mincost:
                mincost = cost
                minidx = key

        return minidx

    def CalcObstMap(self):
        minx = min(self.obstList[:, 0]) - self.params["EXTEND_AREA"]
        maxx = max(self.obstList[:, 0]) + self.params["EXTEND_AREA"]
        miny = min(self.obstList[:, 1]) - self.params["EXTEND_AREA"]
        maxy = max(self.obstList[:, 1]) + self.params["EXTEND_AREA"]

        xwidth = maxx - minx
        xwidth = math.ceil(xwidth / self.gres)
        ywidth = maxy - miny
        ywidth = math.ceil(ywidth / self.gres)

        obmap = np.zeros([xwidth, ywidth])

        for i in range(xwidth):
            ix = minx + (i + 1 / 2) * self.gres
            for j in range(ywidth):
                iy = miny + (j + 1 / 2) * self.gres

                for obsTemp in self.obstList:
                    d = math.sqrt((obsTemp[0] - ix) ** 2 + (obsTemp[1] - iy) ** 2)
                    if d < self.gres:
                        obmap[i][j] = 1
                        break
        return minx, maxx, miny, maxy, obmap

    @staticmethod
    def CalcPolicyMap(Close, xw, yw):
        pmap = np.ones([xw, yw]) * np.inf
        for v in list(Close.values()):
            pmap[v.x, v.y] = v.cost

        return pmap

    @staticmethod
    def show(CostMap):
        gridmap = CostMap
        index = (gridmap == np.inf)
        xw, yw = gridmap.shape
        for i in range(xw):
            for j in range(yw):
                if index[i, j]:
                    gridmap[i, j] = 100

        gridmap = np.divide(gridmap, 100)


        cv2.imshow('img', gridmap)
        cv2.waitKey()



if __name__ == '__main__':
    ObstList, ObstLine, Start, End, ObsNum, ObstLineNum, ObstLineList = GetObs(1, inter=0.1)
    GridAStar = GridAStar(ObstList, [End[0], End[1]], 0.05)
    costmap = GridAStar.FindCostMap()
    GridAStar.show(costmap)
