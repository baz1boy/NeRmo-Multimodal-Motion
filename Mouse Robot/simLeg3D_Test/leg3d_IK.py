import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from leg3d_Joint import compute_footend_3d


def Leg_IK(Ix, Iy, Iz, n):

    q1_guess = 35
    q2_guess = 75
    q3_guess = 10

    q1_sol = np.zeros(n)
    q2_sol = np.zeros(n)
    q3_sol = np.zeros(n)

    for i in range(n):
        dx = Ix[i]
        dy = Iy[i]
        dz = Iz[i]

        def func(q):
            xyz = compute_footend_3d(q[0], q[1], q[2])
            x = xyz[0]
            y = xyz[1]
            z = xyz[2]
            return np.array([dx - x, dy - y, dz - z])

        q_sol = fsolve(func, [q1_guess, q2_guess, q3_guess])
        q1_sol[i] = q_sol[0]
        q2_sol[i] = q_sol[1]
        q3_sol[i] = q_sol[2]

        q1_guess, q2_guess, q3_guess = q_sol[0], q_sol[1], q_sol[2]

    return q1_sol, q2_sol, q3_sol