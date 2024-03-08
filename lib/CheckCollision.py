import numpy as np
import math


class CheckCollision:
    def __init__(self, Vehicle, Config):
        self.Vehicle = Vehicle
        self.Config = Config

        self.LB = self.Vehicle.lr  # distance from rear to vehicle back end
        self.LF = self.Vehicle.lw + self.Vehicle.lf  # distance from rear to vehicle front end
        self.LW = self.Vehicle.lb  # width of vehicle

        self.buffer_bias = 0.5
        self.WBUBBLE_DIST = (self.LB + self.LF)/2.0 - self.LB + self.buffer_bias  # [m] distance from rear and the center of whole bubble
        # self.WBUBBLE_R = (self.LB + self.LF)/2.0  # [m] whole bubble radius
        self.WBUBBLE_R = math.hypot((self.LB + self.LF) / 2.0, self.LW / 2.0) + 0.1  # [m] whole bubble radius
        self.VRX = [self.LF, self.LF, -self.LB, -self.LB, self.LF]
        self.VRY = [-self.LW/2.0, self.LW/2.0, self.LW/2.0, -self.LW/2.0, -self.LW/2.0]
    
    def circle_layer_check(self, ix, iy, iyaw, ox, oy):
        c = math.cos(-iyaw)
        s = math.sin(-iyaw)

        for (iox, ioy) in zip(ox, oy):
            tx = iox - ix
            ty = ioy - iy
            lx = (c * tx - s * ty)  # 将障碍物坐标坐标变换到车体坐标系
            ly = (s * tx + c * ty)

            for (ciecle_r, circle_x, circle_y) in zip(self.Vehicle.circle_radius, self.Vehicle.circle_x, self.Vehicle.circle_y):
                if(math.hypot((circle_x - lx), (circle_y - ly))) < ciecle_r:
                    return False
        return True

    def rect_check(self, ix, iy, iyaw, ox, oy):
        c = math.cos(-iyaw)
        s = math.sin(-iyaw)

        for (iox, ioy) in zip(ox, oy):
            tx = iox - ix
            ty = ioy - iy
            lx = (c * tx - s * ty)  # 将障碍物坐标坐标变换到车体坐标系
            ly = (s * tx + c * ty)

            sumangle = 0.0
            for i in range(len(self.VRX) - 1):
                x1 = self.VRX[i] - lx
                y1 = self.VRY[i] - ly
                x2 = self.VRX[i + 1] - lx
                y2 = self.VRY[i + 1] - ly
                d1 = math.hypot(x1, y1)
                d2 = math.hypot(x2, y2)
                theta1 = math.atan2(y1, x1)
                tty = (-math.sin(theta1) * x2 + math.cos(theta1) * y2)
                tmp = (x1 * x2 + y1 * y2) / (d1 * d2)

                # if tmp >= 1.0:
                #     tmp = 1.0
                # elif tmp <= 0.0:
                #     tmp = 0.0
                #
                # if tty >= 0.0:
                #     sumangle += math.acos(tmp)
                # else:
                #     sumangle -= math.acos(tmp)

                sumangle += abs(math.acos(tmp))

            if sumangle >= (2 * math.pi):
                return False

            # if sumangle >= math.pi:
            #     return False  # collision
        return True  # OK

    def check_collision(self, ox, oy, x, y, yaw, kdtree):
        for (ix, iy, iyaw) in zip(x, y, yaw):

            cx = ix + self.WBUBBLE_DIST * math.cos(iyaw)  # 计算圆点x
            cy = iy + self.WBUBBLE_DIST * math.sin(iyaw)  # 计算圆点y

            # Whole bubble check
            ids = kdtree.search_in_distance([cx, cy], self.WBUBBLE_R)  # 查找距离圆点 为 R的所有的点
            # println(length(ids))
            # print len(ids)
            if len(ids) == 0:
                continue

            temp_ox, temp_oy = [], []
            for i in ids:
                temp_ox.append(ox[i])
                temp_oy.append(oy[i])

            if self.circle_layer_check(ix, iy, iyaw, temp_ox, temp_oy) == False:
                return False  # collision
            # println(ids)

        # plt.plot(x, y, ".k")
        # plt.plot(ox, oy, ".k")
        # plt.show()

        return True  # OK
    
    def varifyIndex(self, x, y, Config):
        for i in range(len(x)):
            # print(x[i], y[i])
            if math.ceil((x[i] - Config["MINX"])/Config["XY_GRID_RESOLUTION"]) < 0:
                return False
            elif math.ceil((x[i] - Config["MINX"])/Config["XY_GRID_RESOLUTION"]) > Config["XWIDTH"]:
                return False
            elif math.ceil((y[i] - Config["MINY"])/Config["XY_GRID_RESOLUTION"]) < 0:
                return False
            elif math.ceil((y[i] - Config["MINY"])/Config["XY_GRID_RESOLUTION"]) > Config["YWIDTH"]:
                return False

        return True


