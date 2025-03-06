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

	t_init = 200
	q_phase = np.array([np.linspace(init_diff[i], q_init[i], t_init) for i in range(14)]).T
	for i in range(t_init+500):
		if i<(t_init):
			ctrlData = np.radians(q_phase[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_phase[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		
    ## Surmounting an Obstacle Motion
	p0 = np.array([[0.0325, 0.0039325, -0.08], [-0.0325, 0.0039325, -0.08], 
				 	[0.0325, 0.0039325, -0.08], [-0.0325, 0.0039325, -0.08]])
	p1 = np.array([[0.0325, -0.0160675, -0.08], [-0.0325, -0.0160675, -0.08], 
					[0.0325, -0.0160675, -0.08], [-0.0325, -0.0160675, -0.08]])		
	pm = (p0+p1)/2
	qm = p2q(pm)
	q_head = np.array([0,0])
	q_start = np.hstack((q_head, qm))

    ## Start up
	t_startup = 200
	q_startup = np.array([np.linspace(q_phase[-1][i], q_start[i], int(t_startup/2)) for i in range(14)]).T
	for i in range(t_startup):
		if i<(t_startup/2):
			ctrlData = np.radians(q_startup[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_startup[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	pleg_cur = q2p(q_startup[-1])

	h_lift = 0.005
	q_move = RC.moveControl(p0, p1, h_lift)
	n = len(q_move)

	t_transtion = 200
	q_transiton = np.array([np.linspace(q_startup[-1][i], q_move[0][i], int(t_transtion/2)) for i in range(14)]).T
	for i in range(t_transtion):
		if i<(t_transtion/2):
			ctrlData = np.radians(q_transiton[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_transiton[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	t_move = 1500
	q_reach = np.zeros_like(q_transiton[-1])
	for i in range(t_move):
		stepNUM = i % n
		ctrlData = np.radians(q_move[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		top = robot.topPosition()
		#print(top)

		# Obstacle detected:
		# reach_obstacle, _, _ = robot.surmountingPos()
		# if(reach_obstacle)<=-0.075: #-0.08
		# 	q_reach = q_move[stepNUM]
		# 	break
	
	t_pause = 200
	for i in range(t_pause):
		ctrlData = np.radians(q_move[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	# move1:
	pleg_cur = q2p(q_reach)
	q_move1 = RC.surmountControl(pleg_cur, 1)
	t_move1 = 1000
	for i in range(t_move1):
		stepNUM = i % len(q_move1)
		ctrlData = np.radians(q_move1[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	pleg_cur2 = q2p(q_move1[stepNUM])

	# move2:
	q_move2 = RC.surmountControl(pleg_cur2,2)
	t_move2 = 600
	for i in range(t_move2):
		stepNUM = i % len(q_move2)
		ctrlData = np.radians(q_move2[stepNUM])
		q_continue = q_move2[stepNUM]
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	
	t_pause = 100
	for i in range(t_pause):
		ctrlData = np.radians(q_continue)
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	pleg_cur3 = q2p(q_continue)

	# move3: 
	q_move3 = RC.surmountControl(pleg_cur3,3)
	t_adj = 200
	q_adj = np.array([np.linspace(q_continue[i], q_move3[0][i], int(t_adj)) for i in range(14)]).T
	for i in range(t_adj+100):
		if i<(t_adj):
			ctrlData = np.radians(q_adj[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_adj[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	t_move3 = 1000
	#print(333)
	for i in range(t_move3):
		stepNUM = i % len(q_move3)
		ctrlData = np.radians(q_move3[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	pleg_cur4 = q2p(q_move3[stepNUM])

	# move 4:
	q_move4 = RC.surmountControl(pleg_cur4,4)
	t_move4 = 500
	#print(444)
	for i in range(t_move4):
		stepNUM = i % len(q_move4)
		ctrlData = np.radians(q_move4[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	t_pause = 100
	for i in range(t_pause):
		ctrlData = np.radians(q_move4[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	pleg_cur5 = q2p(q_move4[stepNUM])

	q_move6 = RC.surmountControl(pleg_cur5,6)
	t_move6 = 1400
	#print(666)
	for i in range(t_move6):
		stepNUM = i % len(q_move6)
		ctrlData = np.radians(q_move6[stepNUM])
		q_continue = q_move6[stepNUM]
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	t_pause = 100
	for i in range(t_pause):
		ctrlData = np.radians(q_continue)
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	pleg_cur7 = q2p(q_continue)

	# move 7:
	q_move7 = RC.surmountControl(pleg_cur7,7)
	t_adj = 200
	q_adj = np.array([np.linspace(q_continue[i], q_move7[0][i], t_adj) for i in range(14)]).T
	for i in range(t_adj+200):
		if i<(t_adj):
			ctrlData = np.radians(q_adj[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_adj[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	t_move7 = 3000
	for i in range(t_move7):
		stepNUM = i % len(q_move7)
		ctrlData = np.radians(q_move7[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

		#FL Reach the ground
		reach_fl, _ = robot.footPos()
		if reach_fl < 0.01:
			q_reach = q_move7[stepNUM]
			break

	# move 8:
	pleg_cur8 = q2p(q_reach)
	q_move8 = RC.surmountControl(pleg_cur8,8)
	t_move8 = 3000
	for i in range(t_move8):
		stepNUM = i % len(q_move8)
		ctrlData = np.radians(q_move8[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

		#HL Reach the ground
		_, reach_hl = robot.footPos()
		if reach_hl < 0.01:
			q_reach = q_move8[stepNUM]
			break

	pleg_cur9 = q2p(q_reach)
	q_move9 = RC.surmountControl(pleg_cur9,9)
	t_move9 = 1000
	for i in range(t_move9):
		stepNUM = i % len(q_move9)
		ctrlData = np.radians(q_move9[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()