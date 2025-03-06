import numpy as np
from leg_Data import Legfl, Legfr, Leghl, Leghr

L0 = Leghl.L0
L1 = Leghl.L1; L2 = Leghl.L2
L3 = Leghl.L3; L4 = Leghl.L4
L6 = Leghl.L6; alpha1 = Leghl.alpha1
L5 = Leghl.L5; alpha2 = Leghl.alpha2

L7 = Leghl.L7; L8 =Leghl.L8
L9 = Leghl.L9
L10 = Leghl.L10     #foot
gamma = Leghl.gamma

def compute_footend_2d(q1, q2):
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

def compute_joint_2d(q1, q2):
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
    
    A = np.array([Ax, Ay])
    B = np.array([Bx, By])
    C = np.array([Cx, Cy])
    D = np.array([Dx, Dy])
    E = np.array([Ex, Ey])
    F = np.array([Fx, Fy])
    G = np.array([Gx, Gy])
    H = np.array([Hx, Hy])

    return A, B, C, D, E, F, G, H

def computefootend_l(q1, q2, q3):
    Iy, Iz = compute_footend_2d(q1, q2)
    I = rotationleft(q3, Iy, Iz)

    return I

def computefootend_r(q1, q2, q3):
    Iy, Iz = compute_footend_2d(q1, q2)
    I = rotationright(q3, Iy, Iz)

    return I

def computerjoint_l(q1, q2, q3):
    A2d, B2d, C2d, D2d, E2d, F2d, G2d, H2d = compute_joint_2d(q1, q2)
    
    A3d = rotationleft(q3, A2d[0], A2d[1])
    B3d = rotationleft(q3, B2d[0], B2d[1])
    C3d = rotationleft(q3, C2d[0], C2d[1])
    D3d = rotationleft(q3, D2d[0], D2d[1])
    E3d = rotationleft(q3, E2d[0], E2d[1])
    F3d = rotationleft(q3, F2d[0], F2d[1])
    G3d = rotationleft(q3, G2d[0], G2d[1])
    H3d = rotationleft(q3, H2d[0], H2d[1])

    return A3d, B3d, C3d, D3d, E3d, F3d, G3d, H3d

def computerjoint_r(q1, q2, q3):
    A2d, B2d, C2d, D2d, E2d, F2d, G2d, H2d = compute_joint_2d(q1, q2)
    
    A3d = rotationright(q3, A2d[0], A2d[1])
    B3d = rotationright(q3, B2d[0], B2d[1])
    C3d = rotationright(q3, C2d[0], C2d[1])
    D3d = rotationright(q3, D2d[0], D2d[1])
    E3d = rotationright(q3, E2d[0], E2d[1])
    F3d = rotationright(q3, F2d[0], F2d[1])
    G3d = rotationright(q3, G2d[0], G2d[1])
    H3d = rotationright(q3, H2d[0], H2d[1])

    return A3d, B3d, C3d, D3d, E3d, F3d, G3d, H3d

def rotationleft(theta, py, pz):
    """
    :param theta
    :param py
    :param pz
    """
    d_hip = 0.0325


    p = np.array([[d_hip], [py], [pz]])

    R = np.array([[np.cos(np.radians(-theta)), 0, np.sin(np.radians(-theta))], 
                  [0, 1, 0], 
                  [-np.sin(np.radians(-theta)), 0, np.cos(np.radians(-theta))]])
    
    P = np.dot(R, p)
  
    return P.flatten()

def rotationright(theta, py, pz):
    """
    :param theta
    :param py
    :param pz
    """
    d_hip = 0.0325


    p = np.array([[-d_hip], [py], [pz]])

    R = np.array([[np.cos(np.radians(theta)), 0, np.sin(np.radians(theta))], 
                  [0, 1, 0], 
                  [-np.sin(np.radians(theta)), 0, np.cos(np.radians(theta))]])
    
    P = np.dot(R, p)
  
    return P.flatten()


# theta = -30  
# py = 10    
# pz = 20     

# P = rotation(theta, py, pz)
# print(P)

