import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv
import time
import math
x_bias=0
y_bias=0

def show(path, case, path_num):
    # plt.figure()
    fig, ax = plt.subplots() 
    # plt.subplot(1, 2, 1)
    plt.xlim(case.xmin, case.xmax)
    plt.ylim(case.ymin, case.ymax)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.gca().set_axisbelow(True)
    plt.title('Case %d' % (path_num))
    plt.grid(linewidth=0.2)
    plt.xlabel('X / m', fontsize=14)
    plt.ylabel('Y / m', fontsize=14)
    for j in range(0, case.obs_num):
        plt.fill(case.obs[j][:, 0], case.obs[j][:, 1], facecolor='k', alpha=0.5)

    temp = case.vehicle.create_polygon(case.x0 , case.y0, case.theta0)
    plt.plot(temp[:, 0], temp[:, 1], linestyle='--', linewidth=0.4, color='green')

    for (ciecle_r, circle_x, circle_y) in zip(case.vehicle.circle_radius, case.vehicle.circle_x, case.vehicle.circle_y):
        delta_x = circle_x * math.cos(case.theta0) + circle_y * math.sin(case.theta0)
        delta_y = circle_x * math.sin(case.theta0) - circle_y * math.cos(case.theta0)
        circle_tmp = patches.Circle((delta_x+case.x0, delta_y+case.y0), ciecle_r, edgecolor='red', facecolor='none')
        ax.add_patch(circle_tmp)
    temp = case.vehicle.create_polygon(case.xf, case.yf, case.thetaf)
    plt.plot(temp[:, 0], temp[:, 1], linestyle='--', linewidth=0.4, color='red')

    for i in range(len(path.x)):
        temp = case.vehicle.create_polygon(path.x[i], path.y[i], path.yaw[i])
        plt.plot(temp[:, 0]-x_bias, temp[:, 1]-y_bias, linestyle='--', linewidth=0.4, color='blue')
        plt.plot(path.x[i]-x_bias, path.y[i]-y_bias, marker='.', color='red', markersize=0.5)

    plt.savefig("./Result/traj{}.jpg".format(path_num))
    plt.show()

