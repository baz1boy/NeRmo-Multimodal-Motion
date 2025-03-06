import numpy as np
from math import *


def cubic_bezier(y0, yf, x):
    assert 0 <= x <= 1, "t must be between 0 and 1"
    y_diff = yf - y0
    bezier = x**3 + 3 * x**2 * (1 - x)
    return y0 + bezier * y_diff

def swing_trajectory(phase, p0, pf, height):
    p = cubic_bezier(p0[:2], pf[:2], phase) 
    if phase < 0.5:
        zp = cubic_bezier(p0[2], p0[2] + height, phase * 2)
    else:
        zp = cubic_bezier(p0[2] + height, pf[2], phase * 2 - 1)
    return np.array([*p, zp])

def stance_trajectory(phase, p0, pf, height0=0):
    p = cubic_bezier(p0[:2], pf[:2], phase) 
    if phase < 0.5:
        zp = cubic_bezier(p0[2], p0[2] + height0, phase * 2)
    else:
        zp = cubic_bezier(p0[2] + height0, pf[2], phase * 2 - 1)
    return np.array([*p, zp])

def push_trajectory(phase, p0, pf, height):
    p = cubic_bezier(p0[:2], pf[:2], phase) 
    if phase < 0.5:
        zp = cubic_bezier(p0[2], p0[2] + height, phase * 2)
    else:
        zp = cubic_bezier(p0[2] + height, pf[2], phase * 2 - 1)
    return np.array([*p, zp])




