import numpy as np
from scipy.optimize import fsolve
from joint_position import compute_foot_end_position

class servo_value(object):

    def q_values(self, start_point, cycle, time_step):

        # # Target position of I
        # Iy_target = np.linspace(-44.1162, -5, 100)
        # Iz_target = -67.9463 * np.ones(len(Iy_target))

        # Desired trajectory - Part 1: Straight line
        y_start = start_point[0]
        z_start = start_point[1]
        k = 0.5
        s = 0.040
        stepNum_down = int(cycle*k/time_step)
        y1 = np.linspace(y_start, y_start + s, stepNum_down)
        z1 = z_start * np.ones_like(y1)

        # Desired trajectory - Part 2: Cycloid (Counter clockwise in negative y-axis direction)
        stepNum_up = int(cycle*(1-k)/time_step)
        r = s / (2 * np.pi)                         # radius of the cycloid         trajectory h = 2r
        t = np.linspace(0, 2 * np.pi, stepNum_up)
        y2 = y_start + s - r * (t - np.sin(t))      # starting point in y direction
        z2 = z_start + r * (1 - np.cos(t))          # starting point in z direction
        z2 = np.flip(z2)                            # Reverse the order of the cycloid part of the trajectory

        # Combine the trajectories
        Iy_target = np.concatenate((y1, y2))
        Iz_target = np.concatenate((z1, z2))
        
        steps = stepNum_up + stepNum_down

        # Initial guesses for q1 and q2
        q1_guess = 42
        q2_guess = 72
        q1_sol = np.zeros_like(Iy_target)
        q2_sol = np.zeros_like(Iz_target)

        for i in range(len(Iy_target)):
            dx = Iy_target[i]
            dy = Iz_target[i]

            def func(q):
                x, y = compute_foot_end_position(q[0], q[1])
                return [dx - x, dy - y]

            q_sol = fsolve(func, [q1_guess, q2_guess])
            q1_sol[i] = q_sol[0]
            q2_sol[i] = q_sol[1]

            q1_guess, q2_guess = q_sol[0], q_sol[1]

        # print("Calculated q1 values:", q1_sol[5])
        # print("Calculated q2 values:", q2_sol[10])

        return q1_sol, q2_sol, steps
    

    
  
