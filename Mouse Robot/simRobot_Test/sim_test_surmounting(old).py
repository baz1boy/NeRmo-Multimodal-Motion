from matplotlib import pyplot as plt
from sim_control import simModel
from sim_controller import robotController, q2p, p2q
import numpy as np


INIT_STEPS = 2000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	robot = simModel("models/robot_body_surmounting.xml")
	RC = robotController()
	T = 2
	timeStep = 0.002

	init_diff = RC.init_qdiff  	# due to model setup, modeling data
	pleg = RC.init_pleg
	pbody = RC.init_pbody
	angle = RC.init_angle
	center = RC.init_center
	#print(pbody)

	## initialization
	q0 = np.array([0,0])
	q_init = RC.init_q
	q_init = np.hstack((q0,q_init))
	init_diff = np.hstack((q0,init_diff))

	t_init = 500
	q_phase = np.array([np.linspace(init_diff[i], q_init[i], t_init) for i in range(14)]).T
	for i in range(t_init):
		ctrlData = np.radians(q_phase[i])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		
    ## Surmounting an Obstacle Motion
	p0 = np.array([[0.0325, 0.0039325, -0.050], [-0.0325, 0.0039325, -0.050], 
				 	[0.0325, 0.0039325, -0.050], [-0.0325, 0.0039325, -0.050]])
	p1 = np.array([[0.0325, -0.0160675, -0.050], [-0.0325, -0.0160675, -0.050], 
					[0.0325, -0.0160675, -0.050], [-0.0325, -0.0160675, -0.050]])		
	pm = (p0+p1)/2
	qm = p2q(pm)
	q_head = np.array([0,0])
	q_start = np.hstack((q_head, qm))

    ## Start up
	t_startup = 200
	q_startup = np.array([np.linspace(q_phase[-1][i], q_start[i], int(t_startup)) for i in range(14)]).T
	for i in range(t_startup+500):
		if i<(t_startup):
			ctrlData = np.radians(q_startup[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_startup[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	pleg_cur = q2p(q_startup[-1])

	# h_lift = 0.005
	# q_move = RC.moveControl(p0, p1, h_lift)
	# n = len(q_move)

	# t_transtion = 200
	# q_transiton = np.array([np.linspace(q_startup[-1][i], q_move[0][i], int(t_transtion/2)) for i in range(14)]).T
	# #print(q_transiton[0])
	# for i in range(t_transtion):
	# 	if i<(t_transtion/2):
	# 		ctrlData = np.radians(q_transiton[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_transiton[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# t_move = 5000
	# q_reach = np.zeros_like(q_transiton[-1])
	# for i in range(t_move):
	# 	stepNUM = i % n
	# 	ctrlData = np.radians(q_move[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# 	top = robot.topPosition()
	# 	#print(top)

	# 	# Obstacle detected:
	# 	reach_obstacle, _, _ = robot.surmountingPos()
	# 	if(reach_obstacle)<=-0.08: #-0.08
	# 		q_reach = q_move[stepNUM]
	# 		break
	
	# t_pause = 200
	# for i in range(t_pause):
	# 	ctrlData = np.radians(q_reach)
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# # adjust posture
	# t_adj1 = 200
	# q_adj1 = np.array([np.linspace(q_reach[i], q_startup[-1][i], int(t_adj1)) for i in range(14)]).T
	# for i in range(t_adj1+500):
	# 	if i<(t_adj1):
	# 		ctrlData = np.radians(q_adj1[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_adj1[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# pleg_cur0 = q2p(q_adj1[-1])
	
	# # move0:
	# q_move0 = RC.surmountControl(pleg_cur0,0)
	# t_move0 = len(q_move0)
	# for i in range(t_move0+500):
	# 	if i<(t_move0):
	# 		ctrlData = np.radians(q_move0[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_move0[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# pleg_cur1 = q2p(q_move0[-1])
	
	# # move1:
	# q_move1 = RC.surmountControl(pleg_cur1,1)
	# t_move1 = len(q_move1)
	# for i in range(t_move1+500):
	# 	if i<(t_move1):
	# 		ctrlData = np.radians(q_move1[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_move1[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# pleg_cur2 = q2p(q_move1[-1])
	
	# # move2:
	# q_move2 = RC.surmountControl(pleg_cur2,2)
	# t_move2 = 5000
	# q_reach = np.zeros_like(q_move1[-1])
	# for i in range(t_move2):
	# 	stepNUM = i % len(q_move2)
	# 	ctrlData = np.radians(q_move2[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# 	# Obstacle detected:
	# 	_, shoulder_reach, _ = robot.surmountingPos()
	# 	if(shoulder_reach)<=-0.005: #-0.005
	# 		q_reach = q_move2[stepNUM]
	# 		break
	
	# t_pause = 200
	# for i in range(t_pause):
	# 	ctrlData = np.radians(q_reach)
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# q_adj = q_reach.copy()
	# q_adj[4] = 0
	# q_adj[7] = 0
	# t_adj = 100
	# q_adj = np.array([np.linspace(q_reach[i], q_adj[i], int(t_adj)) for i in range(14)]).T
	# for i in range(t_adj+200):
	# 	if i<(t_adj):
	# 		ctrlData = np.radians(q_adj[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_adj[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# pleg_cur3 = q2p(q_adj[-1])

	# # move3: 
	# q_move3 = RC.surmountControl(pleg_cur3,3)
	# t_move3 = 5000
	# for i in range(t_move3):
	# 	stepNUM = i % len(q_move3)
	# 	ctrlData = np.radians(q_move3[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
		
	# 	# Obstacle detected:
	# 	_, _, hip_reach = robot.surmountingPos()
	# 	if(hip_reach)<=0.02: #0.02
	# 		q_reachhip = q_move3[stepNUM]
	# 		break
	
	# t_pause = 200
	# for i in range(t_pause):
	# 	ctrlData = np.radians(q_reachhip)
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# pleg_cur4 = q2p(q_reachhip)


	# # q_move4 = RC.surmountControl(pleg_cur4,4)
	# # t_move4 = len(q_move4)
	# # for i in range(t_move4+500):
	# # 	if i<(t_move4):
	# # 		ctrlData = np.radians(q_move4[i])
	# # 		robot.ctrlServo(ctrlData, timeStep)
	# # 	else:
	# # 		ctrlData = np.radians(q_move4[-1])
	# # 		robot.ctrlServo(ctrlData, timeStep)

	# # move 4:
	# q_move4 = RC.surmountControl(pleg_cur4,4)
	# t_move4 = 1000
	# for i in range(t_move4):
	# 	stepNUM = i % len(q_move4)
	# 	ctrlData = np.radians(q_move4[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# pleg_cur5 = q2p(q_move4[0])

	# move 5:
	# q_move5 = RC.surmountControl(pleg_cur5,5)
	# t_move5 = 1000
	# for i in range(t_move5):
	# 	stepNUM = i % len(q_move5)
	# 	ctrlData = np.radians(q_move5[stepNUM])
	# 	q_continue = q_move5[stepNUM]
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# t_pause = 100
	# for i in range(t_pause):
	# 	ctrlData = np.radians(q_continue)
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# pleg_cur6 = q2p(q_continue)

	# # move 6:
	# q_move6 = RC.surmountControl(pleg_cur6,6)
	# t_move6 = 1500
	# for i in range(t_move6):
	# 	stepNUM = i % len(q_move6)
	# 	ctrlData = np.radians(q_move6[stepNUM])
	# 	q_continue = q_move6[stepNUM]
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# t_pause = 100
	# for i in range(t_pause):
	# 	ctrlData = np.radians(q_continue)
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# # move 7:
	# p7 = np.array([[0.0325, 0.004, -0.05], [-0.0325, 0.004, -0.05], 
	# 				[0.0325, 0.004, -0.05], [-0.0325, 0.004, -0.05]])
	# q7= p2q(p7)
	# q_head = np.array([0,0])
	# q7 = np.hstack((q_head, q7))
	# t_adj = 200
	# q_adj = np.array([np.linspace(q_continue[i], q7[i], t_adj) for i in range(14)]).T
	# for i in range(t_adj+200):
	# 	if i<(t_adj):
	# 		ctrlData = np.radians(q_adj[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_adj[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# pleg_cur7 = q2p(q_adj[-1])
	# q_move7 = RC.surmountControl(pleg_cur7,7)
	# t_move7 = 3000
	# for i in range(t_move7):
	# 	stepNUM = i % len(q_move7)
	# 	ctrlData = np.radians(q_move7[stepNUM])
	# 	q_continue = q_move7[stepNUM]
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()