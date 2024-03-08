import numpy as np


def deg2rad(deg):
    return np.pi * (deg/180)


def mod2pi(v):
    # v = rad % 2 * np.pi
    # if v < -np.pi:
    #     v = v + 2 * np.pi
    # elif v > np.pi:
    #     v = v - 2 * np.pi

    while v > np.pi:
        v = v - 2.0 * np.pi

    while v < -np.pi:
        v = v + 2.0 * np.pi

    return v

