import numpy as np
import math
import lib.RSPath as rs
from lib.util import *
from scipy import spatial
import scipy.spatial
from lib.CheckCollision import CheckCollision


class Node(object):
    # xind::Int64  # x index
    # yind::Int64  # y index
    # yawind::Int64  # yaw index
    # direction::Bool  # moving direction forword:true, backword:false
    # x::Array
    # {Float64}  # x position [m]
    # y::Array
    # {Float64}  # y position [m]
    # yaw::Array
    # {Float64}  # yaw angle [rad]
    # yaw1::Array
    # {Float64}  # trailer yaw angle [rad]
    # directions::Array
    # {Bool}  # directions of each points forward: true, backward:false
    # steer::Float64  # steer input
    # cost::Float64  # cost
    # pind::Int64  # parent index
    def __init__(self, xind, yind, yawind, direction, x, y, yaw, directions, steer, cost, pind):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.x = x
        self.y = y
        self.yaw = yaw
        self.directions = directions
        self.steer = steer
        self.cost = cost
        self.pind = pind


class Path(object):

    # x::Array{Float64} # x position [m]
    # y::Array{Float64} # y position [m]
    # yaw::Array{Float64} # yaw angle [rad]
    # yaw1::Array{Float64} # trailer angle [rad]
    # direction::Array{Bool} # direction forward: true, back false
    # cost::Float64 # cost

    def __init__(self, x, y, yaw, direction, cost):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction
        self.cost = cost


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
        Search NN
        inp: input data, single frame or multi frame
        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """
        index = self.tree.query_ball_point(inp, r)
        return index


class HybridAStar:
    def __init__(self, Start, End, Vehicle, Config):
        self.Start = Start
        self.End = End
        self.Vehicle = Vehicle
        self.Config = Config
        self.CheckCollision = CheckCollision(self.Vehicle, self.Config)

    def findPath(self):
        syaw0 = mod2pi(self.Start[2])
        gyaw0 = mod2pi(self.End[2])

        ox = self.Config["ObstList"][:, 0]
        oy = self.Config["ObstList"][:, 1]
        self.tox, self.toy = ox[:], oy[:]
        kdtree = KDTree(np.vstack((ox, oy)).T)

        sxidx, syidx, syawidx = self.calGridIndex(self.Start[0], self.Start[1], self.Start[2])
        exidx, eyidx, eyawidx = self.calGridIndex(self.End[0], self.End[1], self.End[2])

        nstart = Node(sxidx, syidx, syawidx, True,
                      [self.Start[0]], [self.Start[1]], [self.Start[2]], [True], 0, 0, "-1,-1,-1")

        ngoal = Node(exidx, eyidx, eyawidx, True,
                     [self.End[0]], [self.End[1]], [self.End[2]], [True], 0, 0, "-1,-1,-1")

        OpenList = {}
        CloseList = {}
        fnode = None
        u, d = self.getMotionInputs()

        OpenList[self.getIndex(nstart.xind, nstart.yind, nstart.yawind)] = nstart

        assert self.CheckCollision.check_collision(ox, oy, nstart.x, nstart.y, nstart.yaw,
                                                   kdtree) is True, "Error in Start Node!"
        assert self.CheckCollision.check_collision(ox, oy, ngoal.x, ngoal.y, ngoal.yaw,
                                                   kdtree) is True, "Error in Goal Node!"

        times = 0
        while 1:
            if len(OpenList) == 0:
                print("Error: Cannot find path, no open set...")
                return []

            # 从OpenList中取出损失最小得节点作为当前节点，并将其放入CloseList中
            c_id = self.popOpen(OpenList)
            currentNode = OpenList[c_id]
            del OpenList[c_id]
            CloseList[c_id] = currentNode

            # visualize_x = []
            # visualize_y = []
            # for v in closedset.values():
            #     visualize_x.append(v.x[-1])
            #     visualize_y.append(v.y[-1])
            # print visualize_x,visualize_y
            # plt.plot(tox, toy, ".k")
            # plt.plot(visualize_x,visualize_y,'.k')
            # plt.pause(0.1)

            isupdated, fpath = self.updateNodeWithAnalysticExpantion(currentNode, ngoal, kdtree)
            if isupdated:
                print("Find RSpath x,y is")
                print(currentNode.xind, currentNode.yind)
                print(currentNode.x, currentNode.y)
                fnode = fpath
                break

            for i in range(len(u)):

                node = self.calNextNode(currentNode, c_id, u[i], d[i])

                if self.verifyIndex(node, kdtree) is False:
                    continue

                node_ind = self.getIndex(node.xind, node.yind, node.yawind)

                if node_ind in CloseList:
                    continue

                if node_ind not in OpenList:
                    if self.Config["Anchoring"]:
                        if self.if_anchor_done(node):
                            return self.getFinalPath(CloseList, node, nstart)
                        else:
                            OpenList[node_ind] = node
                    else:

                        OpenList[node_ind] = node
                else:
                    if OpenList[node_ind].cost > node.cost:
                        OpenList[node_ind] = node
                times += 1

        print("Final expand node:{}".format(len(OpenList)+len(CloseList)))
        path = self.getFinalPath(CloseList, fnode, nstart)
        print("iterNum:", times)
        return path
            
    def getFinalPath(self, closelist, ngoal, nstart):
        rx, ry, ryaw = ngoal.x[::-1], ngoal.y[::-1], ngoal.yaw[::-1]
        direction = ngoal.directions[::-1]
        nid = ngoal.pind
        finalcost = ngoal.cost

        while 1:
            n = closelist[nid]
            rx.extend(n.x[::-1])
            ry.extend(n.y[::-1])
            ryaw.extend(n.yaw[::-1])
            direction.extend(n.directions[::-1])
            nid = n.pind
            if self.is_same_grid(n, nstart):
                break

        rx = rx[::-1]
        ry = ry[::-1]
        ryaw = ryaw[::-1]
        direction = direction[::-1]

        # adjuct first direction
        direction[0] = direction[1]

        path = Path(rx, ry, ryaw, direction, finalcost)

        return path

    def updateNodeWithAnalysticExpantion(self, cnode, ngoal, kdtree):
        if self.Config["Anchoring"]:
            apath = None
        else:
            apath = self.analysticExpantion(cnode, ngoal, kdtree)
        if apath != None:
            fx = apath.x[1:]
            fy = apath.y[1:]
            fyaw = apath.yaw[1:]
            fcost = cnode.cost + self.calRSPathCost(apath)
            fpind = self.getIndex(cnode.xind, cnode.yind, cnode.yawind)

            fd = []
            for d in apath.directions[1:]:
                if d >= 0:
                    fd.append(True)
                else:
                    fd.append(False)

            fsteer = 0.0
            fpath = Node(cnode.xind, cnode.yind, cnode.yawind, cnode.direction, fx, fy, fyaw, fd, fsteer, fcost, fpind)
            return True, fpath
        return False, None

    def analysticExpantion(self, cnode, ngoal, kdtree):
        sx = cnode.x[-1]
        sy = cnode.y[-1]
        syaw = cnode.yaw[-1]

        # 最大曲率=最小转弯半径的倒数
        max_curvature = 1 / self.Vehicle.MIN_CIRCLE

        if self.Config["Anchoring"]:
            paths = []
            for other_i in [0]:
                for other_j in [0]:
                    paths_temp = rs.calc_paths(sx, sy, syaw, ngoal.x[-1] + other_i*0.2, ngoal.y[-1] + other_j*0.2, ngoal.yaw[-1],
                                  max_curvature, step_size=self.Config["MOTION_RESOLUTION"])
                    paths.extend(paths_temp)

        else:
            paths = rs.calc_paths(sx, sy, syaw, ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1],
                                  max_curvature, step_size=self.Config["MOTION_RESOLUTION"])

        if len(paths) == 0:
            return None

        pathset = {}
        path_id = 0

        if self.Config["MUST_SHORTEST"]:
            # ==============默认为方式二 ==========================================
            # ==============使用寻路方式一：优先找最短路径之后判断可行性=================
            for path in paths:
                pathset[path_id] = path
                path_id = path_id + 1

            for i in range(len(pathset)):
                p_id = min(
                    pathset, key=lambda o: self.calRSPathCost(pathset[o])
                )
                path = pathset[p_id]
                # if draw == True:
                #     plt.grid(True)
                #     plt.axis("equal")
                #     plt.plot(path.x, path.y, linewidth = '0.3', color= 'red')
                #     plt.pause(0.01)
                if self.CheckCollision.check_collision(self.tox, self.toy, path.x, path.y, path.yaw, kdtree):
                    # plt.plot(path.x, path.y, "-^b")
                    return path
        else:
            # ==============使用寻路方式二：优先找可行路径之后选取最短路径=================
            for path in paths:
                if self.CheckCollision.check_collision(self.tox, self.toy, path.x, path.y, path.yaw, kdtree):
                    pathset[path_id] = path
                    path_id = path_id + 1

            if len(pathset) > 0:
                p_id = min(
                    pathset, key=lambda o: self.calRSPathCost(pathset[o])
                )

                path = pathset[p_id]

                return path
            else:
                return None

    def calNextNode(self, cnode, c_id, u, d):
        arc_l = self.Config["XY_GRID_RESOLUTION"] * 1.5  # 保证每次移动的下一个位置在Grid的下一个方格中
        nlist = math.ceil(arc_l / self.Config["MOTION_RESOLUTION"]) + 1

        xlist, ylist, yawlist = [], [], []

        xlist_0 = cnode.x[-1] + d * self.Config["MOTION_RESOLUTION"] * math.cos(cnode.yaw[-1])
        ylist_0 = cnode.y[-1] + d * self.Config["MOTION_RESOLUTION"] * math.sin(cnode.yaw[-1])
        yawlist_0 = mod2pi(cnode.yaw[-1] + d * self.Config["MOTION_RESOLUTION"] / self.Vehicle.lw * math.tan(u))
        xlist.append(xlist_0)
        ylist.append(ylist_0)
        yawlist.append(yawlist_0)

        for i in range(1, int(nlist)):
            xlist_i = xlist[i - 1] + d * self.Config["MOTION_RESOLUTION"] * math.cos(yawlist[i-1])
            ylist_i = ylist[i - 1] + d * self.Config["MOTION_RESOLUTION"] * math.sin(yawlist[i - 1])
            yawlist_i = mod2pi(yawlist[i - 1] + d * self.Config["MOTION_RESOLUTION"] / self.Vehicle.lw * math.tan(u))
            xlist.append(xlist_i)
            ylist.append(ylist_i)
            yawlist.append(yawlist_i)

        xind = math.floor((xlist[-1] - self.Config["MINX"]) / self.Config["XY_GRID_RESOLUTION"])
        yind = math.floor((ylist[-1] - self.Config["MINY"]) / self.Config["XY_GRID_RESOLUTION"])
        yawind = math.floor((yawlist[-1] - self.Config["MINYAW"]) / self.Config["YAW_GRID_RESOLUTION"])

        addcost = 0.0
        if d > 0:
            direction = True
            addcost += abs(arc_l)
        else:
            direction = False
            addcost += abs(arc_l) * self.Config["BACK_COST"] * 0.1

        if direction != cnode.direction:
            addcost += self.Config["SB_COST"]*10

        addcost += abs(u) * self.Config["STEER_COST"]

        addcost += abs(cnode.steer - u) * self.Config["STEER_CHANGE_COST"]

        # if self.Config["Anchoring"]:
        #     # 使其尽量靠近锚点
        #     addcost -= abs(yawlist[-1] - self.End[-1]) * 1000

        cost = cnode.cost + addcost

        directions = [direction for _ in range(len(xlist))]
        node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, directions, u, cost, c_id)

        return node

    def calGridIndex(self, x, y, yaw):
        gres = self.Config["XY_GRID_RESOLUTION"]
        yawres = self.Config["YAW_GRID_RESOLUTION"]

        xidx = math.floor((x - self.Config["MINX"]) / gres)
        yidx = math.floor((y - self.Config["MINY"]) / gres)
        yaw = mod2pi(yaw)
        yawidx = math.floor((yaw - self.Config["MINYAW"]) / yawres)

        if(x == 10):
            a = 1

        assert 0 <= xidx < self.Config["XWIDTH"], f"Error {x} in x bound!"
        assert 0 <= yidx < self.Config["YWIDTH"], f"Error {y} in y bound!"

        assert self.Config["ObstMap"][xidx, yidx] != np.inf, "Error in obs!"

        return xidx, yidx, yawidx

    def getIndex(self, xidx, yidx, yawidx):
        # xidx, yidx, yawidx = self.calGridIndex(x, y, yaw)
        return str(xidx) + "-" + str(yidx) + "-" + str(yawidx)

    def calHCost(self, node):
        cnode_cost = node.cost
        cxidx = math.floor((node.x[-1] - self.Config["MINX"]) / self.Config["XY_GRID_RESOLUTION"])
        cyidx = math.floor((node.y[-1] - self.Config["MINY"]) / self.Config["XY_GRID_RESOLUTION"])
        hcost = self.Config["ObstMap"][cxidx, cyidx]

        return cnode_cost + self.Config["H_COST"] * hcost

    def calRSPathCost(self, rspath):
        cost = 0.0

        for l in rspath.lengths:
            if l >= 0:  # forward
                cost += l
            else:  # back
                cost + abs(l) * self.Config["BACK_COST"]

        # switch back penalty
        for i in range(len(rspath.lengths)-1):
            if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:  # switch back
                cost += self.Config["SB_COST"]

        # steer penalty
        for ctype in rspath.ctypes:
            if ctype != "S":
                cost += abs(self.Vehicle.MAX_STEER) * self.Config["STEER_COST"]

        # steer change penalty
        nctypes = len(rspath.ctypes)
        ulist = [0.0 for _ in range(nctypes)]
        for i in range(nctypes):
            if rspath.ctypes[i] == "R":
                ulist[i] = - self.Vehicle.MAX_STEER
            elif rspath.ctypes[i] == "L":
                ulist[i] = self.Vehicle.MAX_STEER

        for i in range(len(rspath.ctypes) - 1):
            cost += self.Config["STEER_CHANGE_COST"] * abs(ulist[i + 1] - ulist[i])

        return cost

    def getMotionInputs(self):
        up = []
        for i in range(int(self.Config["N_STEER"]) - 1, -1, -1):
            x = self.Vehicle.MAX_STEER - i * (self.Vehicle.MAX_STEER / self.Config["N_STEER"])
            up.append(x)
        # print up
        if self.Config["LEFT_FIRST"]:
            u = [0.0] + [i for i in up] + [-i for i in up]
        else:
            u = [0.0] + [-i for i in up] + [i for i in up]
        # print u
        if self.Config["GO_FIRST"]:
            d = [1.0 for i in range(len(u))] + [-1.0 for i in range(len(u))]
        else:
            d = [-1.0 for i in range(len(u))] + [1.0 for i in range(len(u))]
        # print d
        u = u + u
        # print u
        return u, d

    def vehicleDynamic(self, x, y, yaw, D, delta):
        x = x + D * math.cos(yaw)
        y = y + D * math.sin(yaw)
        yaw = yaw + (D * math.tan(delta)) / self.Vehicle.lw
        yaw = mod2pi(yaw)
        return x, y, yaw

    def verifyIndex(self, node, kdtree):
        if math.ceil((node.x[-1] - self.Config["MINX"])/self.Config["XY_GRID_RESOLUTION"]) < 0:
            return False
        elif math.ceil((node.x[-1] - self.Config["MINX"])/self.Config["XY_GRID_RESOLUTION"]) > self.Config["XWIDTH"]:
            return False
        elif math.ceil((node.y[-1] - self.Config["MINY"])/self.Config["XY_GRID_RESOLUTION"]) < 0:
            return False
        elif math.ceil((node.y[-1] - self.Config["MINY"])/self.Config["XY_GRID_RESOLUTION"]) > self.Config["YWIDTH"]:
            return False

        if self.CheckCollision.check_collision(self.tox, self.toy, node.x, node.y, node.yaw, kdtree) is False:
            return False

        return True

    def popOpen(self, openList):
        min_cost = np.inf
        min_idx = None
        for key in openList.keys():
            cnode = openList[key]
            cnode_cost = self.calHCost(cnode)
            if cnode_cost < min_cost:
                min_cost = cnode_cost
                min_idx = key

        assert min_idx is not None, "Error in open node!"

        return min_idx

    def if_anchor_done(self, cnode):
        anchor_xmin = self.End[0] - 0.5
        anchor_xmax = self.End[0] + 0.5
        anchor_ymin = self.End[1] - 0.5
        anchor_ymax = self.End[1] + 0.5
        anchor_yawmin = self.End[2] - math.pi/4
        anchor_yawmax = self.End[2] + math.pi/4
        if anchor_xmin < cnode.x[-1] < anchor_xmax and anchor_ymin < cnode.y[-1] < anchor_ymax \
                and anchor_yawmin < cnode.yaw[-1] < anchor_yawmax:
            return True
        else:
            return False

    @staticmethod
    def is_same_grid(node1, node2):

        if node1.xind != node2.xind:
            return False

        if node1.yind != node2.yind:
            return False

        if node1.yawind != node2.yawind:
            return False

        return True

