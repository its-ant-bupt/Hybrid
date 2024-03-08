import numpy as np
import math

'''
circle radius
1, 1, 1.100000023841858, 1.0499999523162842, 1, 0.5199999809265137, 0.5199999809265137, 0.3499999940395355, 0.3499999940395355, 0.11249999701976776, 0.11249999701976776, 0.11249999701976776, 0.11249999701976776
circle x
3.1500000953674316, -0.20000000298023224, 2.25, 1.2999999523162842, 0.4000000059604645, -0.699999988079071, -0.699999988079071, 3.6500000953674316, 3.6500000953674316, 3.3931400775909424, 3.3931400775909424, -0.17749999463558197, -0.17749999463558197
circle y
0, 0, 0, 0, 0, -0.4000000059604645, 0.4000000059604645, 0.6000000238418579, -0.6000000238418579, 0.8100000023841858, -0.8100000023841858, 0.8199999928474426, -0.8199999928474426
circle layer
0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1

const carDimensions = {
  boundingBoxPosition: [-1.3, 0.0, 0.0],
  centerOfMass: [1.556544, 0.0, 0.58],
  height: 1.503,
  inertia: [925.0, 5093.0, 5524.0],
  length: 5.393,
  mass: 2417.0,
  width: 2.109,
  widthWithoutMirrors: 1.954,
};
'''

class Vehicle:
    def __init__(self):
        self.lw = 3.165  # wheelbase
        self.lf = 0.83  # front hang length
        self.lr = 1.396  # rear hang length
        self.lb = 1.954  # width
        self.MAX_STEER = 0.75  # 方向盘最大转角
        # self.MIN_CIRCLE = self.lw / math.tan(self.MAX_STEER)  # 车辆最小转角
        self.MIN_CIRCLE = 1/0.2
        self.circle_radius, self.circle_x, self.circle_y = self.make_collision_buffer()
    
    def make_collision_buffer(self):
        circle_radius = [1, 1, 1.100000023841858, 1.0499999523162842, 
                         1, 0.5199999809265137, 0.5199999809265137, 0.5499999940395355, 0.5499999940395355]
        circle_x = [3.1500000953674316, -0.20000000298023224, 2.25, 1.2999999523162842,
                    0.4000000059604645, -0.699999988079071, -0.699999988079071, 3.6500000953674316, 3.6500000953674316]
        circle_y = [0, 0, 0, 0,
                    0, -0.4000000059604645, 0.4000000059604645, 0.6000000238418579, -0.6000000238418579]
        return circle_radius, circle_x, circle_y

    def create_polygon(self, x, y, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        points = np.array([
            [-self.lr, -self.lb / 2, 1],
            [self.lf + self.lw, -self.lb / 2, 1],
            [self.lf + self.lw, self.lb / 2, 1],
            [-self.lr, self.lb / 2, 1],
            [-self.lr, -self.lb / 2, 1],
        ]).dot(np.array([
            [cos_theta, -sin_theta, x],
            [sin_theta, cos_theta, y],
            [0, 0, 1]
        ]).transpose())
        return points[:, 0:2]