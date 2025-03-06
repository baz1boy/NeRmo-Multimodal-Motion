import numpy as np
from math import *
from body_kinematics import bodyIK
import leg_Gait as gait
from leg_Joint import computefootend_l, computefootend_r

def verticalMove(delta_h, pleg):
    q1_0, q2_0, q3_0, _ = gait.heightControl(pleg[0], delta_h, 0)       # index 0 = left part
    q1_1, q2_1, q3_1, _ = gait.heightControl(pleg[1], delta_h, 1)       # index 1 = right part
    q1_2, q2_2, q3_2, _ = gait.heightControl(pleg[2], delta_h, 0)       
    q1_3, q2_3, q3_3, _ = gait.heightControl(pleg[3], delta_h, 1)

    q_out= np.array([q1_0,q2_0,q3_0, q1_1,q2_1,q3_1, q1_2,q2_2,q3_2, q1_3,q2_3,q3_3]).T

    return q_out

def straightMove(p0_l, pf_l, p0_r, pf_r, h):   
    q1_left_swing, q2_left_swing, q3_left_swing, _ = gait.locomotion(p0_l, pf_l, h, 1, 0)
    q1_left_stance, q2_left_stance, q3_left_stance, _ = gait.locomotion(p0_l, pf_l, h, 0, 0)
    q1_right_swing, q2_right_swing, q3_right_swing, _ = gait.locomotion(p0_r, pf_r, h, 1, 1)
    q1_right_stance, q2_right_stance, q3_right_stance, _ = gait.locomotion(p0_r, pf_r, h, 0, 1)

    q_left_swing = np.array([q1_left_swing, q2_left_swing, q3_left_swing]).T
    q_left_stance = np.array([q1_left_stance, q2_left_stance, q3_left_stance]).T
    q_right_swing = np.array([q1_right_swing, q2_right_swing, q3_right_swing]).T
    q_right_stance = np.array([q1_right_stance, q2_right_stance, q3_right_stance]).T

    # print(q1_left_swing)
    # print(q_left_swing)
    # print(q_left_stance)

    q_left = np.vstack((q_left_stance, q_left_swing))
    q_right = np.vstack((q_right_stance, q_right_swing))

    return q_left, q_right

def turning(direction, pleg, pbody, angle, center):
    """
    param:  pleg = footend pos in leg coordinate before move
            pbody = footend pos in body coordinate

    """
    (omega, phi, psi) = angle
    (xm,ym,zm)=center
    (Tlf,Trf,Tlh,Trh,Tm) = bodyIK(omega,phi,psi, xm,ym,zm)

    ## phase 1:: body move, pbody constant
    Ileg = np.zeros_like(pbody)                  #footend position in leg coordinate after move
    Ileg[0]=np.linalg.inv(Tlf)@pbody[0]          #left fore     
    Ileg[1]=np.linalg.inv(Trf)@pbody[1]          #right fore
    Ileg[2]=np.linalg.inv(Tlh)@pbody[2]          #left hind
    Ileg[3]=np.linalg.inv(Trh)@pbody[3]          #right hind   ""+ left/ccw: [0],[3]      - right turning/cw: [1],[2]  

    q_s = calculateRobotservo(pleg)
    q_f = calculateRobotservo(Ileg)
    #print(q_s, q_f)

    t1 = 50
    q_phase1 = np.array([np.linspace(q_s[i], q_f[i], t1) for i in range(12)]).T
    #print(q_phase1)

    ## phase 2:: consider   +left/ccw  -right/cw
    """
    case 1: leftfore,righthind [0][3] first  
    case 2: rightfore,lefthind [1][2] first
    """
    Ileg_s = pleg[:, :-1]
    Ileg_f = Ileg[:, :-1]
    t2 = 50
    q1_0, q2_0, q3_0, _ = gait.tuning(Ileg_f[0], Ileg_s[0], 0)       # index 0 = left part
    q1_1, q2_1, q3_1, _ = gait.tuning(Ileg_f[1], Ileg_s[1], 1)       # index 1 = right part
    q1_2, q2_2, q3_2, _ = gait.tuning(Ileg_f[2], Ileg_s[2], 0)       
    q1_3, q2_3, q3_3, _ = gait.tuning(Ileg_f[3], Ileg_s[3], 1)
    q_phase2 = np.array([q1_0,q2_0,q3_0, q1_1,q2_1,q3_1, q1_2,q2_2,q3_2, q1_3,q2_3,q3_3]).T
    #print(q_phase2)

    # case 1: [0][3]first
    def expand03(ori_q):
        expandArray = np.zeros((t2*2, 12))

        # set the first 3 and the last 3 columns in the first t2 rows to corresponding original q 
        expandArray[:t2,:3] = ori_q[:,:3]
        expandArray[:t2,-3:] = ori_q[:,-3:]
        # set the last t2 rows with first 3 and last 3 columns's last row 
        expandArray[t2:,:3] = ori_q[-1,:3]
        expandArray[t2:,-3:] = ori_q[-1,-3:]

        # the middle 6 columns of first t2 rows are filled using the first row of original q
        # then the last t2 rows maintain the order of original q
        expandArray[:t2, 3:9] = ori_q[0, 3:9]
        expandArray[t2:, 3:9] = ori_q[:, 3:9]
        return expandArray

    # case 2: [1][2]first
    def expand12(ori_q):
        expandArray = np.zeros((t2*2, 12))

        # set the first 3 and the last 3 columns in the first t2 rows with the first row of original
        expandArray[:t2,:3] = ori_q[0,:3]
        expandArray[:t2,-3:] = ori_q[0,-3:]
        # set the last t2 rows with first 3 and last 3 columns's original q 
        expandArray[t2:,:3] = ori_q[:,:3]
        expandArray[t2:,-3:] = ori_q[:,-3:]

        # the first t2 rows maintain the order of original q
        # the middle 6 columns of last t2 rows are filled using the last row of original q
        expandArray[:t2, 3:9] = ori_q[:, 3:9]
        expandArray[t2:, 3:9] = ori_q[-1, 3:9]
        return expandArray
    
    # case single leg: [0][3][1][2]
    def expand0312(ori_q):
        expandArray = np.zeros((t2*4, 12))
        qleg0 = ori_q[:,:3]
        qleg1 = ori_q[:,3:6]
        qleg2 = ori_q[:,6:9]
        qleg3 = ori_q[:,-3:]

        expandArray[:t2, :3] = qleg0
        expandArray[:t2, 3:6] = qleg1[0,:]
        expandArray[:t2, 6:9] = qleg2[0,:]
        expandArray[:t2, -3:] = qleg3[0,:]

        expandArray[t2:t2*2, :3] = qleg0[-1,:]
        expandArray[t2:t2*2, 3:6] = qleg1[0,:]
        expandArray[t2:t2*2, 6:9] = qleg2[0,:]
        expandArray[t2:t2*2, -3:] = qleg3

        expandArray[t2*2:t2*3, :3] = qleg0[-1,:]
        expandArray[t2*2:t2*3, 3:6] = qleg1
        expandArray[t2*2:t2*3, 6:9] = qleg2[0,:]
        expandArray[t2*2:t2*3, -3:] = qleg3[-1,:]

        expandArray[t2*3:, :3] = qleg0[-1,:]
        expandArray[t2*3:, 3:6] = qleg1[-1,:]
        expandArray[t2*3:, 6:9] = qleg2
        expandArray[t2*3:, -3:] = qleg3[-1,:]

        return expandArray
    
    # case single leg: [1][2][0][3]
    def expand1203(ori_q):
        expandArray = np.zeros((t2*4, 12))
        qleg0 = ori_q[:,:3]
        qleg1 = ori_q[:,3:6]
        qleg2 = ori_q[:,6:9]
        qleg3 = ori_q[:,-3:]

        expandArray[:t2, :3] = qleg0[0,:]
        expandArray[:t2, 3:6] = qleg1
        expandArray[:t2, 6:9] = qleg2[0,:]
        expandArray[:t2, -3:] = qleg3[0,:]

        expandArray[t2:t2*2, :3] = qleg0[0,:]
        expandArray[t2:t2*2, 3:6] = qleg1[-1,:]
        expandArray[t2:t2*2, 6:9] = qleg2
        expandArray[t2:t2*2, -3:] = qleg3[0,:]

        expandArray[t2*2:t2*3, :3] = qleg0
        expandArray[t2*2:t2*3, 3:6] = qleg1[-1,:]
        expandArray[t2*2:t2*3, 6:9] = qleg2[-1,:]
        expandArray[t2*2:t2*3, -3:] = qleg3[0,:]

        expandArray[t2*3:, :3] = qleg0[-1,:]
        expandArray[t2*3:, 3:6] = qleg1[-1,:]
        expandArray[t2*3:, 6:9] = qleg2[-1,:]
        expandArray[t2*3:, -3:] = qleg3

        return expandArray
    
    if direction == 0:
        q_phase2_expand = expand03(q_phase2)
    elif direction == 1:
        q_phase2_expand = expand12(q_phase2)
    
    q_out = np.vstack((q_phase1, q_phase2_expand))
    return q_out

def pitchrollyaw(pleg, pbody, angle, center):
    (omega,phi,psi)=angle
    (xm,ym,zm)=center
    (Tlf,Trf,Tlh,Trh,Tm) = bodyIK(omega,phi,psi, xm,ym,zm)
    
    Ileg0=np.linalg.inv(Tlf)@pbody[0]
    Ileg1=np.linalg.inv(Trf)@pbody[1]         
    Ileg2=np.linalg.inv(Tlh)@pbody[2]         
    Ileg3=np.linalg.inv(Trh)@pbody[3]
    Ileg = np.array([Ileg0,Ileg1,Ileg2,Ileg3])

    q_f = calculateRobotservo(Ileg)
    q_s = calculateRobotservo(pleg)

    t = 30
    q_out = np.array([np.linspace(q_s[i], q_f[i], t) for i in range(12)]).T
    #print(q_out[-1])
    
    pleg_cur = calculateRobotfootend(q_out[-1])

    return q_out, pleg_cur

def fallRecovery(pleg, q_init, index):
    q0 = np.array([0,0])
    q_cur = calculateRobotservo(pleg)
    q_cur = np.hstack((q0,q_cur))
    if index==0:    ### robot in a side-lying position
        ### back to default pos
        servo1 = -45
        servo2 = 45
        rot = -35
        q_pos = np.array([0,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0])
        t1 = 100
        q_out1 = np.array([np.linspace(q_cur[i], q_pos[i], t1) for i in range(14)]).T

        q_wait1 = np.vstack([q_pos]*50)

        ### approaching a standing pos
        q_move = np.array([0,0, servo1,servo2,rot, servo1,servo2,rot, servo1,servo2,rot, servo1,servo2,rot])
        t2 = 10   # 5 = 0.1s
        q_out2 = np.array([np.linspace(q_pos[i], q_move[i], t2) for i in range(14)]).T

        q_wait2 = np.vstack([q_move]*50)

        q_stand1 = np.array([0,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0])
        t3 = 100
        q_out3 = np.array([np.linspace(q_move[i], q_stand1[i], t3) for i in range(14)]).T

        q_stand2 = np.array([0,0, 0,servo2,0, 0,servo2,0, 0,servo2,0, 0,servo2,0])
        t33 = 100
        q_out33 = np.array([np.linspace(q_stand1[i], q_stand2[i], t33) for i in range(14)]).T

        t4 = 100
        q_out4 = np.array([np.linspace(q_stand2[i], q_init[i], t4) for i in range(14)]).T

        q_out = np.vstack((q_out1, q_wait1, q_out2, q_wait2, q_out3, q_out33, q_out4))
    elif index==1:
        ###phase 1:: from back-lying to side-lying
        head_lift = 60
        q_pos = np.array([0,0, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_1 = 100
        q_p1_1 = np.array([np.linspace(q_cur[i], q_pos[i], t_p1_1) for i in range(14)]).T

        q_p1_wait1 = np.vstack([q_pos]*100)

        q_headmove1 = np.array([0,head_lift, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_2 = 100
        q_p1_2 = np.array([np.linspace(q_pos[i], q_headmove1[i], t_p1_2) for i in range(14)]).T

        q_p1_wait2 = np.vstack([q_headmove1]*200)

        q_headmove2 = np.array([90,head_lift, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_3 = 50
        q_p1_3 = np.array([np.linspace(q_headmove1[i], q_headmove2[i], t_p1_3) for i in range(14)]).T

        q_p1_wait3 = np.vstack([q_headmove2]*200)

        q_headmove3 = np.array([90,head_lift, 0,50,-60, 0,50,20, 0,50,-60, 0,50,20])
        t_p1_4 = 100
        q_p1_4 = np.array([np.linspace(q_headmove2[i], q_headmove3[i], t_p1_4) for i in range(14)]).T

        q_p1_wait4 = np.vstack([q_headmove3]*200)

        q_headmove4 = np.array([0,head_lift, 0,50,-60, 0,50,20, 0,50,-60, 0,50,20])
        t_p1_5 = 30
        q_p1_5 = np.array([np.linspace(q_headmove3[i], q_headmove4[i], t_p1_5) for i in range(14)]).T

        q_p1_wait5 = np.vstack([q_headmove4]*200)

        q_headmove5 = np.array([0,0, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_6 = 100
        q_p1_6 = np.array([np.linspace(q_headmove4[i], q_headmove5[i], t_p1_6) for i in range(14)]).T

        ###phase 2:: from side-lying to standing
        q_p2_wait0 = np.vstack([q_headmove5]*200)

        servo1 = -45
        servo2 = 45
        rot = -35

        q_pos = np.array([0,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0])
        t_p2_1 = 100
        q_p2_1 = np.array([np.linspace(q_headmove5[i], q_pos[i], t_p2_1) for i in range(14)]).T

        q_p2_wait1 = np.vstack([q_pos]*50)

        q_move = np.array([0,0, servo1,servo2,rot, servo1,servo2,rot, servo1,servo2,rot, servo1,servo2,rot])
        t_p2_2 = 5
        q_p2_2 = np.array([np.linspace(q_pos[i], q_move[i], t_p2_2) for i in range(14)]).T

        q_p2_wait2 = np.vstack([q_move]*50)

        q_stand1 = np.array([0,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0])
        t_p2_3 = 100
        q_p2_3 = np.array([np.linspace(q_move[i], q_stand1[i], t_p2_3) for i in range(14)]).T

        q_stand2 = np.array([0,0, 0,servo2,0, 0,servo2,0, 0,servo2,0, 0,servo2,0])
        t_p2_33 = 100
        q_p2_33 = np.array([np.linspace(q_stand1[i], q_stand2[i], t_p2_33) for i in range(14)]).T

        t_p2_4 = 100
        q_p2_4 = np.array([np.linspace(q_stand2[i], q_init[i], t_p2_4) for i in range(14)]).T

        q_out = np.vstack((q_p1_1, q_p1_wait1, q_p1_2, q_p1_wait2, q_p1_3, q_p1_wait3, q_p1_4, q_p1_wait4, q_p1_5, q_p1_wait5, q_p1_6,
                           q_p2_wait0, q_p2_1, q_p2_wait1, q_p2_2, q_p2_wait2, q_p2_3, q_p2_33, q_p2_4))
        
    elif index==2:
        ###phase 1:: from back-lying to side-lying
        head_lift = 60
        q_pos = np.array([0,0, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_1 = 100
        q_p1_1 = np.array([np.linspace(q_cur[i], q_pos[i], t_p1_1) for i in range(14)]).T

        q_p1_wait1 = np.vstack([q_pos]*100)

        q_headmove1 = np.array([0,head_lift, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_2 = 100
        q_p1_2 = np.array([np.linspace(q_pos[i], q_headmove1[i], t_p1_2) for i in range(14)]).T

        q_p1_wait2 = np.vstack([q_headmove1]*200)

        q_headmove2 = np.array([-90,head_lift, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_3 = 50
        q_p1_3 = np.array([np.linspace(q_headmove1[i], q_headmove2[i], t_p1_3) for i in range(14)]).T

        q_p1_wait3 = np.vstack([q_headmove2]*200)

        q_headmove3 = np.array([-90,head_lift, 0,50,20, 0,50,-60, 0,50,20, 0,50,-60])
        t_p1_4 = 100
        q_p1_4 = np.array([np.linspace(q_headmove2[i], q_headmove3[i], t_p1_4) for i in range(14)]).T

        q_p1_wait4 = np.vstack([q_headmove3]*200)

        q_headmove4 = np.array([0,head_lift, 0,50,20, 0,50,-60, 0,50,20, 0,50,-60])
        t_p1_5 = 30
        q_p1_5 = np.array([np.linspace(q_headmove3[i], q_headmove4[i], t_p1_5) for i in range(14)]).T

        q_p1_wait5 = np.vstack([q_headmove4]*200)

        q_headmove5 = np.array([0,0, 0,50,0, 0,50,0, 0,50,0, 0,50,0])
        t_p1_6 = 100
        q_p1_6 = np.array([np.linspace(q_headmove4[i], q_headmove5[i], t_p1_6) for i in range(14)]).T

        ###phase 2:: from side-lying to standing
        q_p2_wait0 = np.vstack([q_headmove5]*200)

        servo1 = -45
        servo2 = 45
        rot = -40

        q_pos = np.array([0,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0])
        t_p2_1 = 100
        q_p2_1 = np.array([np.linspace(q_headmove5[i], q_pos[i], t_p2_1) for i in range(14)]).T

        q_p2_wait1 = np.vstack([q_pos]*50)

        q_move = np.array([0,0, servo1,servo2,rot, servo1,servo2,rot, servo1,servo2,rot, servo1,servo2,rot])
        t_p2_2 = 5
        q_p2_2 = np.array([np.linspace(q_pos[i], q_move[i], t_p2_2) for i in range(14)]).T

        q_p2_wait2 = np.vstack([q_move]*50)

        q_stand1 = np.array([0,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0, servo1,servo2,0])
        t_p2_3 = 100
        q_p2_3 = np.array([np.linspace(q_move[i], q_stand1[i], t_p2_3) for i in range(14)]).T

        q_stand2 = np.array([0,0, 0,servo2,0, 0,servo2,0, 0,servo2,0, 0,servo2,0])
        t_p2_33 = 100
        q_p2_33 = np.array([np.linspace(q_stand1[i], q_stand2[i], t_p2_33) for i in range(14)]).T

        t_p2_4 = 100
        q_p2_4 = np.array([np.linspace(q_stand2[i], q_init[i], t_p2_4) for i in range(14)]).T

        q_out = np.vstack((q_p1_1, q_p1_wait1, q_p1_2, q_p1_wait2, q_p1_3, q_p1_wait3, q_p1_4, q_p1_wait4, q_p1_5, q_p1_wait5, q_p1_6,
                           q_p2_wait0, q_p2_1, q_p2_wait1, q_p2_2, q_p2_wait2, q_p2_3, q_p2_33, q_p2_4))
    return q_out

def pointMove(p0, pf, h, index):
    q1_sw, q2_sw, q3_sw, _ = gait.locomotion(p0, pf, h, 1, index)
    q1_st, q2_st, q3_st, _ = gait.locomotion(p0, pf, 0, 0, index)

    q1 = np.hstack((q1_sw, q1_st))
    q2 = np.hstack((q2_sw, q2_st))
    q3 = np.hstack((q3_sw, q3_st))

    return q1, q2, q3

def leanMove(distance, pleg, pbody, angle, center):
    height = 0.002

    (omega,phi,psi)=angle
    (xm,ym,zm)=center
    (Tlf,Trf,Tlh,Trh,Tm) = bodyIK(omega,phi,psi, xm,ym,zm)

    pbody0 = np.zeros_like(pbody)
    pbodyf = np.zeros_like(pbody)

    for i in range(4):
        pbody0[i] = pbody[i]
        pbodyf[i] = pbody[i]
    
    for i in range(4):
        pbody0[i][1] = pbody0[i][1] + distance/2
        pbodyf[i][1] = pbodyf[i][1] - distance/2

    pbody0 = pbody0[:,:-1]
    pbodyf = pbodyf[:,:-1]
    #print(pbody0)
    #print(pbodyf)
    
    Ibody0_sw = gait.bodyLocomotion(pbody0[0], pbodyf[0], height, 1)        # nx3, 
    Ibody0_st = gait.bodyLocomotion(pbody0[0], pbodyf[0], height, 0)
    Ibody0 = np.vstack((Ibody0_sw, Ibody0_st))

    Ibody1_sw = gait.bodyLocomotion(pbody0[1], pbodyf[1], height, 1)
    Ibody1_st = gait.bodyLocomotion(pbody0[1], pbodyf[1], height, 0)
    Ibody1 = np.vstack((Ibody1_sw, Ibody1_st))

    Ibody2_sw = gait.bodyLocomotion(pbody0[2], pbodyf[2], height, 1)
    Ibody2_st = gait.bodyLocomotion(pbody0[2], pbodyf[2], height, 0)
    Ibody2 = np.vstack((Ibody2_sw, Ibody2_st))

    Ibody3_sw = gait.bodyLocomotion(pbody0[3], pbodyf[3], height, 1)
    Ibody3_st = gait.bodyLocomotion(pbody0[3], pbodyf[3], height, 0)
    Ibody3 = np.vstack((Ibody3_sw, Ibody3_st))

    I_ones = np.ones((len(Ibody0), 1))
    pbody_0 = np.hstack((Ibody0,I_ones))
    pbody_1 = np.hstack((Ibody1,I_ones))
    pbody_2 = np.hstack((Ibody2,I_ones))
    pbody_3 = np.hstack((Ibody3,I_ones))
    #print(pbody_0)

    pleg0 = np.zeros_like(pbody_0)
    pleg1 = np.zeros_like(pbody_1)
    pleg2 = np.zeros_like(pbody_2)
    pleg3 = np.zeros_like(pbody_3)
    n = len(pleg0)
    #print(pleg0, n)

    for i in range(n):
        pleg0[i] = np.linalg.inv(Tlf)@pbody_0[i]
        pleg1[i] = np.linalg.inv(Trf)@pbody_1[i]
        pleg2[i] = np.linalg.inv(Tlh)@pbody_2[i]
        pleg3[i] = np.linalg.inv(Trh)@pbody_3[i]

    pleg0 = pleg0[:,:-1]
    pleg1 = pleg1[:,:-1]
    pleg2 = pleg2[:,:-1]
    pleg3 = pleg3[:,:-1]

    q0 = np.zeros_like(pleg0)
    q1 = np.zeros_like(pleg1)
    q2 = np.zeros_like(pleg2)
    q3 = np.zeros_like(pleg3)

    for j in range(n):
        q0[j][0], q0[j][1], q0[j][2], _ = gait.standControl(pleg0[j], 0)
        q1[j][0], q1[j][1], q1[j][2], _ = gait.standControl(pleg1[j], 1)
        q2[j][0], q2[j][1], q2[j][2], _ = gait.standControl(pleg2[j], 0)
        q3[j][0], q3[j][1], q3[j][2], _ = gait.standControl(pleg3[j], 1)

    return q0, q1, q2, q3

def pointPush(p0, pf, h, index):
    q1_sw, q2_sw, q3_sw, _ = gait.squeezingLoco(p0, pf, h, 1, index)
    q1_st, q2_st, q3_st, _ = gait.squeezingLoco(p0, pf, 0, 0, index)

    q1 = np.hstack((q1_sw, q1_st))
    q2 = np.hstack((q2_sw, q2_st))
    q3 = np.hstack((q3_sw, q3_st))

    return q1, q2, q3

def squeezing(pleg, index):
    if index == 0:
        plegf = np.zeros_like(pleg)
        for i in range(4):
            plegf[i] = pleg[i]
        for j in range(4):
            plegf[j][1] = plegf[j][1] - 0.016
        
        pleg_fore0 = np.array([[0.045, -0.01645, -0.025], [-0.045, -0.01645, -0.025]])
        pleg_foref = np.array([[0.045, -0.03245, -0.025], [-0.045, -0.03245, -0.025]])

        h = 0.008
        q01, q02, q03 = pointMove(pleg_fore0[0], pleg_foref[0], h, 0)
        q11, q12, q13 = pointMove(pleg_fore0[1], pleg_foref[1], h, 1)
        q21, q22, q23 = pointMove(pleg[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(pleg[3], plegf[3], h, 1)

        # for i in range(len(q0)):
        #     q0[i] = np.array([-10, 100, 0])
        #     q1[i] = np.array([-10, 100, 0])

    elif index == 1:
        plegf = np.zeros_like(pleg)
        for i in range(4):
            plegf[i] = pleg[i]
        for j in range(4):
            plegf[j][1] = plegf[j][1] - 0.02
        #print(pleg)
        #print(plegf)
        
        h = 0.01
        h2 = 0.006
        q01, q02, q03 = pointMove(pleg[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(pleg[1], plegf[1], h2, 1)
        q21, q22, q23 = pointMove(pleg[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(pleg[3], plegf[3], h2, 1)

    elif index == 2:
        plegf = np.zeros_like(pleg)
        for i in range(4):
            plegf[i] = pleg[i]
        for j in range(4):
            plegf[j][1] = plegf[j][1] - 0.02
        #print(pleg)
        #print(plegf)
        
        h = 0.008
        q01, q02, q03 = pointMove(pleg[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(pleg[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(pleg[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(pleg[3], plegf[3], h, 1)

    q0 =  np.array([q01,q02,q03]).T
    q1 =  np.array([q11,q12,q13]).T
    q2 =  np.array([q21,q22,q23]).T
    q3 =  np.array([q31,q32,q33]).T

    return q0, q1, q2, q3

def squeezingNarrow(pleg, index):
    if index == 0:
        q_out = calculateRobotservo(pleg)
    elif index == 1:
        q_1 = calculateRobotservo(pleg)
        q_2 = calculateRobotservo(pleg)
        q_2[0] = -10
        q_2[1] = 100
        q_out1 = np.array([np.linspace(q_1[i], q_2[i], 100) for i in range(12)]).T
        q_out1_tem = q_out1[-1]

        q_2[2] = -90
        q_out2 = np.array([np.linspace(q_out1_tem[i], q_2[i], 200) for i in range(12)]).T
        q_out2_tem = q_out2[-1]

        q_2[0] = 35
        q_2[1] = 180
        q_out3 = np.array([np.linspace(q_out2_tem[i], q_2[i], 100) for i in range(12)]).T

        q_out = np.vstack((q_out1, q_out2, q_out3))
    elif index == 2:
        plegf = pleg.copy()
        for i in range(4):
            plegf[i][1] = plegf[i][1] - 0.01
        
        h = 0.005
        q01, q02, q03 = pointMove(pleg[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(pleg[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(pleg[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(pleg[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T

        q0[:,0] = 35
        q0[:,1] = 180
        q0[:,2] = -90

        q_out = np.hstack((q0, q1, q2, q3))
    elif index == 3:
        q_1 = calculateRobotservo(pleg)
        q_1[0] = 35
        q_1[1] = 180
        q_1[2] = -90

        q_2 = q_1.copy()
        q_2[0] = -10
        q_2[1] = 100

        q_out1 = np.array([np.linspace(q_1[i], q_2[i], 100) for i in range(12)]).T
        q_out1_tem = q_out1[-1]

        q_2[2] = 0
        q_out2 = np.array([np.linspace(q_out1_tem[i], q_2[i], 200) for i in range(12)]).T
        q_out2_tem = q_out2[-1]

        q_3 = calculateRobotservo(pleg)
        q_out3 = np.array([np.linspace(q_out2_tem[i], q_3[i], 100) for i in range(12)]).T
        q_out3_tem = q_out3[-1]

        p_forward = pleg.copy()
        for i in range(4):
            p_forward[i][1] = p_forward[i][1] + 0.02
        q_4 = calculateRobotservo(p_forward)
        q_out4 = np.array([np.linspace(q_out3_tem[i], q_4[i], 100) for i in range(12)]).T

        q_out = np.vstack((q_out1, q_out2, q_out3, q_out4))
    elif index == 4:
        q_1 = calculateRobotservo(pleg)
        q_2 = calculateRobotservo(pleg)
        q_2[9] = 10
        #q_2[10] = 100
        q_out1 = np.array([np.linspace(q_1[i], q_2[i], 100) for i in range(12)]).T
        q_out1_tem = q_out1[-1]

        q_2[11] = -95
        q_out2 = np.array([np.linspace(q_out1_tem[i], q_2[i], 200) for i in range(12)]).T
        q_out2_tem = q_out2[-1]

        q_2[9] = 35
        q_2[10] = 180
        q_out3 = np.array([np.linspace(q_out2_tem[i], q_2[i], 100) for i in range(12)]).T

        q_out = np.vstack((q_out1, q_out2, q_out3))
    elif index == 5:
        plegf = pleg.copy()
        for i in range(4):
            plegf[i][1] = plegf[i][1] - 0.01
        
        h = 0.005
        q01, q02, q03 = pointMove(pleg[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(pleg[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(pleg[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(pleg[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T

        q3[:,0] = 35
        q3[:,1] = 180
        q3[:,2] = -95

        q_out = np.hstack((q0, q1, q2, q3))
    elif index == 6:
        q_1 = calculateRobotservo(pleg)
        q_1[9] = 35
        q_1[10] = 180
        q_1[11] = -95

        q_2 = calculateRobotservo(pleg)
        q_2[9] = -10
        q_2[10] = 100

        q_out1 = np.array([np.linspace(q_1[i], q_2[i], 100) for i in range(12)]).T
        q_out1_tem = q_out1[-1]

        q_2[11] = 0
        q_out2 = np.array([np.linspace(q_out1_tem[i], q_2[i], 200) for i in range(12)]).T
        q_out2_tem = q_out2[-1]

        q_3 = calculateRobotservo(pleg)
        q_out3 = np.array([np.linspace(q_out2_tem[i], q_3[i], 100) for i in range(12)]).T
        q_out3_tem = q_out3[-1]

        p_mid = pleg.copy()
        for i in range(4):
            p_mid[i][1] = p_mid[i][1] - 0.01
        q_4 = calculateRobotservo(p_mid)
        q_out4 = np.array([np.linspace(q_out3_tem[i], q_4[i], 100) for i in range(12)]).T

        q_out = np.vstack((q_out1, q_out2, q_out3, q_out4))

    return q_out

def sideMove(n, p1, p2, height, index):

    q1_sw, q2_sw, q3_sw, _ = gait.sideShift(n, p1, p2, height, 1, index)
    q1_st, q2_st, q3_st, _ = gait.sideShift(n, p1, p2, 0, 0, index)

    q1 = np.hstack((q1_sw, q1_st))
    q2 = np.hstack((q2_sw, q2_st))
    q3 = np.hstack((q3_sw, q3_st))

    return q1, q2, q3

def surmountObstacle(pleg_cur, p, index):
    if index == 1:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.01
        plegf[1][1] = plegf[1][1] - 0.01
        plegf[2][1] = plegf[2][1] - 0.01
        plegf[3][1] = plegf[3][1] - 0.01
        
        h = 0.005
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T

        q0[:,1] = 230
        q1[:,1] = 230
        # q0[:,0] = 60
        # q1[50:,0] = 60
        q2[:,2] = 0
        q3[:,2] = 0
        
        q_out = np.hstack((q0, q1, q2, q3))

    elif index == 2:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.01
        plegf[1][1] = plegf[1][1] - 0.01
        plegf[2][1] = plegf[2][1] - 0.02
        plegf[3][1] = plegf[3][1] - 0.02

        h = 0.005
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T
        q0[:,1] = 230
        q1[:,1] = 230
        q0[:,0] = 100
        q1[:,0] = 100
        q0[:,2] = 25
        q1[:,2] = 25
        q2[:,2] = 8
        q3[:,2] = 10
        
        q_out = np.hstack((q0, q1, q2, q3))
    
    elif index == 3:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.01
        plegf[1][1] = plegf[1][1] - 0.01
        plegf[2][1] = plegf[2][1] - 0.01
        plegf[3][1] = plegf[3][1] - 0.01

        h = 0.005
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 = np.array([q01,q02,q03]).T
        q1 = np.array([q11,q12,q13]).T
        q2 = np.array([q21,q22,q23]).T
        q3 = np.array([q31,q32,q33]).T

        q_out = np.hstack((q1, q1, q2, q3))

    elif index == 4:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.01
        plegf[1][1] = plegf[1][1] - 0.01
        plegf[2][1] = plegf[2][1] - 0.01
        plegf[3][1] = plegf[3][1] - 0.01

        h = 0.005
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 = np.array([q01,q02,q03]).T
        q1 = np.array([q11,q12,q13]).T
        q2 = np.array([q21,q22,q23]).T
        q3 = np.array([q31,q32,q33]).T
        q2[:,1] = 230
        q3[:,1] = 230
        # q2[:,0] = 60
        # q3[:,0] = 60
        q0[:,2] = 10
        q1[:,2] = 10

        q_out = np.hstack((q0, q1, q2, q3))
    
    elif index == 6:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.01
        plegf[1][1] = plegf[1][1] - 0.01
        plegf[2][1] = plegf[2][1] - 0.01
        plegf[3][1] = plegf[3][1] - 0.01

        h = 0.005
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T
        q2[:,1] = 230
        q3[:,1] = 230
        # q2[:,0] = 100
        # q3[:,0] = 100
        q0[:,2] = 10
        q1[:,2] = 10

        q_out = np.hstack((q0, q1, q2, q3))
    
    elif index == 7:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.02
        plegf[1][1] = plegf[1][1] - 0.02
        plegf[2][1] = plegf[2][1] - 0.02
        plegf[3][1] = plegf[3][1] - 0.02

        h = 0.01
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T
    
        q_out = np.hstack((q0, q1, q2, q3))

    elif index == 8:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.01
        plegf[1][1] = plegf[1][1] - 0.01
        plegf[2][1] = plegf[2][1] - 0.01
        plegf[3][1] = plegf[3][1] - 0.01

        h = 0.005
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T
    
        q_out = np.hstack((q0, q1, q2, q3))

    elif index == 9:
        plegf = p.copy()
        plegf[0][1] = plegf[0][1] - 0.02
        plegf[1][1] = plegf[1][1] - 0.02
        plegf[2][1] = plegf[2][1] - 0.02
        plegf[3][1] = plegf[3][1] - 0.02

        h = 0.006
        q01, q02, q03 = pointMove(p[0], plegf[0], h, 0)
        q11, q12, q13 = pointMove(p[1], plegf[1], h, 1)
        q21, q22, q23 = pointMove(p[2], plegf[2], h, 0)
        q31, q32, q33 = pointMove(p[3], plegf[3], h, 1)

        q0 =  np.array([q01,q02,q03]).T
        q1 =  np.array([q11,q12,q13]).T
        q2 =  np.array([q21,q22,q23]).T
        q3 =  np.array([q31,q32,q33]).T
    
        q_out = np.hstack((q0, q1, q2, q3))

    return q_out



def calculateRobotservo(pleg):
    q1_0, q2_0, q3_0, Ileg_0 = gait.standControl([pleg[0][0],pleg[0][1],pleg[0][2]],0)
    q1_1, q2_1, q3_1, Ileg_1 = gait.standControl([pleg[1][0],pleg[1][1],pleg[1][2]],1)
    q1_2, q2_2, q3_2, Ileg_2 = gait.standControl([pleg[2][0],pleg[2][1],pleg[2][2]],0)
    q1_3, q2_3, q3_3, Ileg_3 = gait.standControl([pleg[3][0],pleg[3][1],pleg[3][2]],1)
    q_cal = np.array([q1_0,q2_0,q3_0, q1_1,q2_1,q3_1, q1_2,q2_2,q3_2, q1_3,q2_3,q3_3])
    return q_cal

def calculateRobotfootend(q4legs):
    q0,q1,q2,q3 = np.split(q4legs, 4)
    p0 = computefootend_l(q0[0],q0[1],q0[2])
    p1 = computefootend_r(q1[0],q1[1],q1[2])
    p2 = computefootend_l(q2[0],q2[1],q2[2])
    p3 = computefootend_r(q3[0],q3[1],q3[2])
    p0123 = np.array([p0,p1,p2,p3])
    homo = np.ones((4,1))
    p_cal = np.hstack((p0123, homo))

    return p_cal




##### this is only for some little tests and for an animation 
def turning_test(legID, pleg, pbody, angle, center):
    """
    set angle interval = 5 

    """
    (omega, phi, psi) = angle

    step = 5
    n, rest = np.divmod(psi, step)

    # yaw
    (xm,ym,zm)=center
    (Tlf,Trf,Tlh,Trh,Tm) = bodyIK(omega,phi,rest, xm,ym,zm)

    Ileg = np.zeros_like(pbody)                  #footend position in leg coordinate
    Ileg[0]=np.linalg.inv(Tlf)@pbody[0]          #left fore     
    Ileg[1]=np.linalg.inv(Trf)@pbody[1]          #right fore
    Ileg[2]=np.linalg.inv(Tlh)@pbody[2]          #left hind
    Ileg[3]=np.linalg.inv(Trh)@pbody[3]          #right hind   ""right turning/cw: 12       left/ccw: 03 
    Ileg = Ileg[:, :-1]

    if legID==0:
        _q1,_q2,_q3,_Itune = gait.tuning(Ileg[0], pleg[0], 0) # index 0 = left part
    elif legID==1:
        _q1,_q2,_q3,_Itune = gait.tuning(Ileg[1], pleg[1], 1) # index 1 = right part
    elif legID==2:
        _q1,_q2,_q3,_Itune = gait.tuning(Ileg[2], pleg[2], 0) # index 0 = left part
    elif legID==3:
        _q1,_q2,_q3,_Itune = gait.tuning(Ileg[3], pleg[3], 1) # index 1 = right part


    (xm,ym,zm)=center
    (Tlf,Trf,Tlh,Trh,Tm) = bodyIK(omega,phi,step, xm,ym,zm)

    Ileg = np.zeros_like(pbody)                  #footend position in leg coordinate
    Ileg[0]=np.linalg.inv(Tlf)@pbody[0]          #left fore     
    Ileg[1]=np.linalg.inv(Trf)@pbody[1]          #right fore
    Ileg[2]=np.linalg.inv(Tlh)@pbody[2]          #left hind
    Ileg[3]=np.linalg.inv(Trh)@pbody[3]          #right hind   ""right turning/cw: 12       left/ccw: 03 
    Ileg = Ileg[:, :-1]

    if legID==0:
        q1_,q2_,q3_,Itune_ = gait.tuning(Ileg[0], pleg[0], 0) # index 0 = left part
    elif legID==1:
        q1_,q2_,q3_,Itune_ = gait.tuning(Ileg[1], pleg[1], 1) # index 1 = right part
    elif legID==2:
        q1_,q2_,q3_,Itune_ = gait.tuning(Ileg[2], pleg[2], 0) # index 0 = left part
    elif legID==3:
        q1_,q2_,q3_,Itune_ = gait.tuning(Ileg[3], pleg[3], 1) # index 1 = right part

    q1_repeat = np.tile(q1_, (n, 1))
    q2_repeat = np.tile(q2_, (n, 1))
    q3_repeat = np.tile(q3_, (n, 1))
    Itune_repeat = np.tile(Itune_, (n, 1))

    q1 = np.vstack((_q1, q1_repeat))
    q2 = np.vstack((_q2, q2_repeat))
    q3 = np.vstack((_q3, q3_repeat))
    Itune = np.vstack((_Itune, Itune_repeat))

    #print((q1.shape[1]))

    return q1,q2,q3,Itune
