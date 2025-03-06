import numpy as np
from leg_data import Legfl, Legfr, Leghl, Leghr

def compute_foot_end_position(q1, q2):
    # Constants
    d_B = 5.6  #mm
    d_A = 8
    d_CD = 4
    d_AE = 5

    L0 = Leghl.L0
    L1 = Leghl.L1; L2 = Leghl.L2
    L3 = Leghl.L3; L4 = Leghl.L4
    L6 = Leghl.L6; alpha1 = Leghl.alpha1
    L5 = Leghl.L5; alpha2 = Leghl.alpha2

    L7 = Leghl.L7; L8 = Leghl.L8
    L9 = Leghl.L9
    L10 = Leghl.L10 # foot
    gamma = Leghl.gamma

    # Initial positions
    Ax, Ay = 0, 0
    Bx, By = 0, L0

    # Position calculations
    Cx = Bx + L1*np.cos(np.radians(q2))
    Cy = By + L1*np.sin(np.radians(q2))

    Ex = Ax + L4*np.cos(np.radians(180+q1))
    Ey = Ay + L4*np.sin(np.radians(180+q1))

    Fx = Ax + L5*np.cos(np.radians(180+q1+alpha2))
    Fy = Ay + L5*np.sin(np.radians(180+q1+alpha2))

    CE = np.sqrt((Cy-Ey)**2 + (Cx-Ex)**2)
    BE = np.sqrt((By-Ey)**2 + (Bx-Ex)**2)
    beta1 = np.degrees(np.arccos((L1**2 + CE**2 - BE**2) / (2 * L1 * CE)))
    beta2 = np.degrees(np.arccos((L2**2 + CE**2 - L3**2) / (2 * L2 * CE)))

    Dx = Cx - L2*np.sin(np.radians(beta1+beta2+90-q2))
    Dy = Cy - L2*np.cos(np.radians(beta1+beta2+90-q2))

    Hx = (L7/L3)*(Ex-Dx) + Ex
    Hy = (L7/L3)*(Ey-Dy) + Ey

    Gx = Fx - Ex + Hx
    Gy = Fy - Ey + Hy

    lambda1 = np.degrees(np.arccos((Hy-Gy)/L9))
    lambda2 = 180 - gamma - lambda1

    Ix = Hx - L10*np.sin(np.radians(lambda2))
    Iy = Hy - L10*np.cos(np.radians(lambda2))

    return Ix, Iy

def compute_joints_position(q1, q2):
    L0 = Leghl.L0
    L1 = Leghl.L1; L2 = Leghl.L2
    L3 = Leghl.L3; L4 = Leghl.L4
    L6 = Leghl.L6; alpha1 = Leghl.alpha1
    L5 = Leghl.L5; alpha2 = Leghl.alpha2

    L7 = Leghl.L7; L8 =Leghl.L8
    L9 = Leghl.L9
    L10 = Leghl.L10 # foot
    gamma = Leghl.gamma

    # Initial positions
    Ax, Ay = 0, 0
    Bx, By = 0, L0

    # Position calculations
    Cx = Bx + L1*np.cos(np.radians(q2))
    Cy = By + L1*np.sin(np.radians(q2))

    Ex = Ax + L4*np.cos(np.radians(180+q1))
    Ey = Ay + L4*np.sin(np.radians(180+q1))

    Fx = Ax + L5*np.cos(np.radians(180+q1+alpha2))
    Fy = Ay + L5*np.sin(np.radians(180+q1+alpha2))

    CE = np.sqrt((Cy-Ey)**2 + (Cx-Ex)**2)
    BE = np.sqrt((By-Ey)**2 + (Bx-Ex)**2)
    beta1 = np.degrees(np.arccos((L1**2 + CE**2 - BE**2) / (2 * L1 * CE)))
    beta2 = np.degrees(np.arccos((L2**2 + CE**2 - L3**2) / (2 * L2 * CE)))

    Dx = Cx - L2*np.sin(np.radians(beta1+beta2+90-q2))
    Dy = Cy - L2*np.cos(np.radians(beta1+beta2+90-q2))

    Hx = (L7/L3)*(Ex-Dx) + Ex
    Hy = (L7/L3)*(Ey-Dy) + Ey

    Gx = Fx - Ex + Hx
    Gy = Fy - Ey + Hy

    C = [Cx, Cy]
    D = [Dx, Dy]
    E = [Ex, Ey]
    F = [Fx, Fy]
    G = [Gx, Gy]
    H = [Hx, Hy]

    return C, D, E, F, G, H



# # Example usage
# q1 = 42
# q2 = 72 
# Ix, Iy = compute_foot_end_position(q1, q2)
# print(f"Foot end position: Ix = {Ix}, Iy = {Iy}")

