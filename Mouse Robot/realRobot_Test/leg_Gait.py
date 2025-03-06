import numpy as np
from leg_Traj import swing_trajectory as swing, stance_trajectory as stance, push_trajectory as push
from leg_IK import leftLeg_IK, rightLeg_IK


'''
configure various posture controls

'''
def resetControl(p, p_init): 
   

    return 

def standControl(p, index):
    """
    param: index = left(0) / right(1) leg

    """
    p0 = np.array(p)  
    pf = p0

    I_stand = np.array([stance(phase, p0, pf, 0) for phase in np.linspace(0, 1, 10)])
    if index==0:
        q1, q2, q3 = leftLeg_IK(I_stand[:,0], I_stand[:,1], I_stand[:,2], len(I_stand))
    elif index==1:
        q1, q2, q3 = rightLeg_IK(I_stand[:,0], I_stand[:,1], I_stand[:,2], len(I_stand))

    return q1[0], q2[0], q3[0], I_stand[0]

def heightControl(p, delta_h, index):
    p0 = np.array(p)  
    pf = p + np.array([0, 0, delta_h])

    I_hCtrl = np.array([stance(phase, p0, pf, 0) for phase in np.linspace(0, 1, 100)])
    if index==0:
        q1, q2, q3 = leftLeg_IK(I_hCtrl[:,0], I_hCtrl[:,1], I_hCtrl[:,2], len(I_hCtrl))
    elif index==1:
        q1, q2, q3 = rightLeg_IK(I_hCtrl[:,0], I_hCtrl[:,1], I_hCtrl[:,2], len(I_hCtrl))

    return q1, q2, q3, I_hCtrl

def tuning(p1, p2, index):
    p0 = p1
    pf = p2
    height = 0.01
    n = 50 

    I_tune = np.array([swing(phase, p0, pf, height) for phase in np.linspace(0, 1, n)])
    if index==0:
        q1, q2, q3 = leftLeg_IK(I_tune[:,0], I_tune[:,1], I_tune[:,2], len(I_tune))
    elif index==1:
        q1, q2, q3 = rightLeg_IK(I_tune[:,0], I_tune[:,1], I_tune[:,2], len(I_tune))

    return q1, q2, q3, I_tune

def sideShift(n, p1, p2, height, sw1_st0, index):
    p0 = p1 
    pf = p2

    # p0 = np.array(p)  
    # pf = p + np.array([delta_x, 0, 0])
    # height = 0.005

    if sw1_st0 == 1:     # swing phase
        I_side = np.array([swing(phase, p0, pf, height) for phase in np.linspace(0, 1, int(n/2))])
        if index == 0:
            q1, q2, q3 = leftLeg_IK(I_side[:,0], I_side[:,1], I_side[:,2], len(I_side))
        elif index == 1:
            q1, q2, q3 = rightLeg_IK(I_side[:,0], I_side[:,1], I_side[:,2], len(I_side))

    elif sw1_st0 == 0:   # stance phase
        I_side = np.array([stance(phase, pf, p0, 0) for phase in np.linspace(0, 1, int(n/2))])
        if index == 0:
            q1, q2, q3 = leftLeg_IK(I_side[:,0], I_side[:,1], I_side[:,2], len(I_side))
        elif index == 1:
            q1, q2, q3 = rightLeg_IK(I_side[:,0], I_side[:,1], I_side[:,2], len(I_side))

    return q1, q2, q3, I_side

def locomotion(p1, p2, height, sw1_st0, index):
    p0 = p1
    pf = p2
    n = 50

    if sw1_st0 == 1:     # swing phase
        I_loco = np.array([swing(phase, p0, pf, height) for phase in np.linspace(0, 1, n)])
        if index == 0:
            q1, q2, q3 = leftLeg_IK(I_loco[:,0], I_loco[:,1], I_loco[:,2], len(I_loco))
        elif index == 1:
            q1, q2, q3 = rightLeg_IK(I_loco[:,0], I_loco[:,1], I_loco[:,2], len(I_loco))

    elif sw1_st0 == 0:   # stance phase
        I_loco = np.array([stance(phase, pf, p0, 0) for phase in np.linspace(0, 1, n)])
        if index == 0:
            q1, q2, q3 = leftLeg_IK(I_loco[:,0], I_loco[:,1], I_loco[:,2], len(I_loco))
        elif index == 1:
            q1, q2, q3 = rightLeg_IK(I_loco[:,0], I_loco[:,1], I_loco[:,2], len(I_loco))

    return q1, q2, q3, I_loco

def bodyLocomotion(p1, p2, height, sw1_st0):
    p0 = p1
    pf = p2
    n = 100

    if sw1_st0 == 1:     # swing phase
        I_out = np.array([swing(phase, p0, pf, height) for phase in np.linspace(0, 1, n)])
    elif sw1_st0 == 0:   # stance phase
        I_out = np.array([stance(phase, pf, p0, 0) for phase in np.linspace(0, 1, n)])

    return I_out

def squeezingLoco(p1, p2, height, sw1_st0, index):
    p0 = p1
    pf = p2
    n = 100

    if sw1_st0 == 1:     # swing phase
        I_push = np.array([swing(phase, p0, pf, height) for phase in np.linspace(0, 1, n)])
        if index == 0:
            q1, q2, q3 = leftLeg_IK(I_push[:,0], I_push[:,1], I_push[:,2], len(I_push))
        elif index == 1:
            q1, q2, q3 = rightLeg_IK(I_push[:,0], I_push[:,1], I_push[:,2], len(I_push))

    elif sw1_st0 == 0:   # push phase
        I_push = np.array([push(phase, pf, p0, 0.005) for phase in np.linspace(0, 1, n)])
        if index == 0:
            q1, q2, q3 = leftLeg_IK(I_push[:,0], I_push[:,1], I_push[:,2], len(I_push))
        elif index == 1:
            q1, q2, q3 = rightLeg_IK(I_push[:,0], I_push[:,1], I_push[:,2], len(I_push))

    return q1, q2, q3, I_push