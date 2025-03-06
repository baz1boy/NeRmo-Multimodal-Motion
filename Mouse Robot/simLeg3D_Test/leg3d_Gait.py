import numpy as np
from leg3d_Traj import swing_trajectory as swing, stance_trajectory as stance
from leg3d_IK import Leg_IK

'''
configure various posture controls

'''
def resetControl(p, p_init): 
    p0 = np.array(p)
    pf = np.array(p_init)

    I_reset = np.array([swing(phase, p0, pf, 0.001) for phase in np.linspace(0, 1, 100)])
    q1, q2, q3 = Leg_IK(I_reset[:,0], I_reset[:,1], I_reset[:,2], len(I_reset))

    return q1, q2, q3, I_reset

def heightControl(delta_h, p):
    p0 = np.array(p)  
    pf = p + np.array([0, 0, delta_h])

    I_hCtrl = np.array([stance(phase, p0, pf, 0) for phase in np.linspace(0, 1, 100)])
    q1, q2, q3 = Leg_IK(I_hCtrl[:,0], I_hCtrl[:,1], I_hCtrl[:,2], len(I_hCtrl))

    return q1, q2, q3, I_hCtrl

def sideShift(delta_x, p, sw1_st0):
    p0 = np.array(p)  
    pf = p + np.array([delta_x, 0, 0])

    height = 0.005

    if sw1_st0 == 1:  # swing phase
        I_side = np.array([swing(phase, p0, pf, height) for phase in np.linspace(0, 1, 100)])
        q1, q2, q3 = Leg_IK(I_side[:,0], I_side[:,1], I_side[:,2], len(I_side))
    else:             # stance phase
        I_side = np.array([stance(phase, pf, p0, 0) for phase in np.linspace(0, 1, 100)])
        q1, q2, q3 = Leg_IK(I_side[:,0], I_side[:,1], I_side[:,2], len(I_side))

    return q1, q2, q3, I_side

def locomotion(p1, p2, sw1_st0):
    p0 = p1
    pf = p2
    height = 0.01

    if sw1_st0 == 1:     # swing phase
        I_loco = np.array([swing(phase, p0, pf, height) for phase in np.linspace(0, 1, 100)])
        q1, q2, q3 = Leg_IK(I_loco[:,0], I_loco[:,1], I_loco[:,2], len(I_loco))
    else:   # stance phase
        I_loco = np.array([stance(phase, pf, p0, 0) for phase in np.linspace(0, 1, 100)])
        q1, q2, q3 = Leg_IK(I_loco[:,0], I_loco[:,1], I_loco[:,2], len(I_loco))

    return q1, q2, q3, I_loco