from matplotlib import pyplot as plt
from sim_control import simModel
from sim_controller import robotController, q2p, p2q
import numpy as np

INIT_STEPS = 2000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	robot = simModel("models/robot_body_squeezing.xml")
	RC = robotController()
	T = 2
	timeStep = 0.002

	init_diff = RC.init_qdiff  	# due to model setup, modeling data
	pleg = RC.init_pleg
	pbody = RC.init_pbody
	angle = RC.init_angle
	center = RC.init_center
	step_diff = RC.stepDiff
	#print(stepdiff)

	## initialization
	q0 = np.array([0,0])
	q_init = RC.init_q
	q_init = np.hstack((q0,q_init))
	init_diff = np.hstack((q0,init_diff))
	#robot.ctrlViewer()

	t_init = 500
	q_phase = np.array([np.linspace(init_diff[i], q_init[i], int(t_init/2)) for i in range(14)]).T
	for i in range(t_init):
		if i<(t_init/2):
			ctrlData = np.radians(q_phase[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_phase[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	
	### Trajectory Point
	p0 = np.array([[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05], 
				 	[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05]])
	p1 = np.array([[0.0325, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05], 
					[0.0325, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05]])		
	pm = (p0+p1)/2
	qm = p2q(pm)
	q_head = np.array([0,0])
	q_start = np.hstack((q_head, qm))

    ## Start up
	t_startup = 500
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


	## Start Moving
	h_lift = 0.005
	q_move = RC.moveControl(p0, p1, h_lift)
	n = len(q_move)
	#print(q_move)
	#print(n)

	# ### Move Test
	# t_move = 6000
	# h_top = []
	# t_top = []
	# for i in range(t_move):
	# 	stepNUM = i % n
	# 	ctrlData = np.radians(q_move[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# 	top = robot.topPosition()
	# 	#print(top)
	# 	h_top.append(top*1000)
	# 	t_top.append(i*0.002)

	# plt.plot(t_top, h_top, lw='2.5')
	# plt.title('Robot Height', fontsize=45)
	# plt.xlabel('Time(s)', fontsize=35)
	# plt.ylabel('Robot Top(mm)', fontsize=35)
	# plt.xticks(fontsize=30)
	# plt.yticks(fontsize=30)
	# #plt.legend(fontsize='large', loc='upper right', shadow=True, fancybox=True)
	# plt.show()

	t_transtion = 500
	q_transiton = np.array([np.linspace(q_startup[-1][i], q_move[0][i], int(t_transtion/2)) for i in range(14)]).T
	#print(q_transiton[0])
	for i in range(t_transtion):
		if i<(t_transtion/2):
			ctrlData = np.radians(q_transiton[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_transiton[-1])
			robot.ctrlServo(ctrlData, timeStep)
	
	### Squeezing Test
	t_move = 5000
	q_reach = np.zeros_like(q_transiton[-1])
	for i in range(t_move):
		stepNUM = i % n
		ctrlData = np.radians(q_move[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		top = robot.topPosition()
		#print(top)

		# hole detected:
		reach_door = robot.squeezingPosition()
		if(reach_door)<=0.04: #0.04
			q_reach = q_move[stepNUM]
			break

	t_pause = 200
	for i in range(t_pause):
		ctrlData = np.radians(q_reach)
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	# bow head:
	# t_tran2squ = 500
	# q_squ1 = RC.squeezingControl(0)
	# q_tran2squ = np.array([np.linspace(q_reach[i], q_squ1[0][i], int(t_tran2squ/2)) for i in range(14)]).T
	# #print()
	# for i in range(t_tran2squ):
	# 	if i<(t_tran2squ/2):
	# 		ctrlData = np.radians(q_tran2squ[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_tran2squ[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# t_squ1 = 5000
	# for i in range(t_squ1):
	# 	stepNUM = i % len(q_squ1)
	# 	ctrlData = np.radians(q_squ1[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# 	# head passed:
	# 	reach_door = robot.squeezingPosition()
	# 	if(reach_door)<=-0.1:
	# 		q_passed_head = q_squ1[stepNUM]
	# 		step_passed_head = i
	# 		print(step_passed_head)
	# 		break

	# # crawling move:
	# h_top = []
	# t_top = []
	# q_squ2 = RC.squeezingControl(1)

	# t_1to2 = 400
	# q_1to2 = np.array([np.linspace(q_passed_head[i], q_squ2[0][i], int(t_1to2/2)) for i in range(14)]).T
	# for i in range(t_1to2):
	# 	if i<(t_1to2/2):
	# 		ctrlData = np.radians(q_1to2[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_1to2[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# t_squ2 = 8000
	# for i in range(t_squ2):
	# 	stepNUM = i % len(q_squ2)
	# 	ctrlData = np.radians(q_squ2[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	# 	top = robot.spinePosition()
	# 	h_top.append(top*1000)
	# 	t_top.append(i*0.002)

	# 	# torso passed:
	# 	passed_torso = robot.holePosition()
	# 	if(passed_torso)<=0:
	# 		q_passed_torso = q_squ2[stepNUM]
	# 		step_passed_torso = i
	# 		print(step_passed_torso)
	# 		break

	# # motion traj adjust
	# q_squ3 = RC.squeezingControl(2)

	# t_2to3 = 200
	# q_2to3 = np.array([np.linspace(q_passed_torso[i], q_squ3[0][i], int(t_1to2/2)) for i in range(14)]).T
	# #print()
	# for i in range(t_2to3):
	# 	if i<(t_2to3/2):
	# 		ctrlData = np.radians(q_2to3[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = np.radians(q_2to3[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()
	
	# t_squ3 = 2000
	# for i in range(t_squ3):
	# 	stepNUM = i % len(q_squ3)
	# 	ctrlData = np.radians(q_squ3[stepNUM])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	robot.ctrlViewer()

	# plt.plot(t_top, h_top, lw='2.5')
	# plt.title('Robot Height', fontsize=45)
	# plt.xlabel('Time(s)', fontsize=35)
	# plt.ylabel('Robot Top(mm)', fontsize=35)
	# plt.xticks(fontsize=30)
	# plt.yticks(fontsize=30)
	# plt.legend(fontsize='large', loc='upper right', shadow=True, fancybox=True)
	# plt.show()
	
	"""
	Through a narrow hole
	"""
	# adjust posture
	t_adj1 = 500
	q_adj1 = RC.squeezingnarrowControl(0)

	q_nar1 = np.array([np.linspace(q_reach[i], q_adj1[i], int(t_adj1/2)) for i in range(14)]).T
	for i in range(t_adj1):
		if i<(t_adj1/2):
			ctrlData = np.radians(q_nar1[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_nar1[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	q_adj2 = RC.squeezingnarrowControl(1)
	t_adj2 = len(q_adj2)
	for i in range(t_adj2+500):
		if i<(t_adj2):
			ctrlData = np.radians(q_adj2[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_adj2[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	# First half
	q_adj3 = RC.squeezingnarrowControl(2)
	t_2to3 = 400
	q_2to3 = np.array([np.linspace(q_adj2[-1][i], q_adj3[0][i], int(t_2to3/2)) for i in range(14)]).T
	for i in range(t_2to3):
		if i<(t_2to3/2):
			ctrlData = np.radians(q_2to3[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_2to3[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	
	t_adj3 = 10000
	for i in range(t_adj3):
		stepNUM = i % len(q_adj3)
		ctrlData = np.radians(q_adj3[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		#half body passed
		half_passed = robot.spineholePos()
		if half_passed <= 0:
			q_half_passed = q_adj3[stepNUM]
			break
			
	# transitional phase, adjust pos
	q_adj4 = RC.squeezingnarrowControl(3)
	t_3to4 = 400
	q_3to4 = np.array([np.linspace(q_half_passed[i], q_adj4[0][i], int(t_3to4/2)) for i in range(14)]).T
	for i in range(t_3to4):
		if i<(t_3to4/2):
			ctrlData = np.radians(q_3to4[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_3to4[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	
	t_adj4 = len(q_adj4)
	for i in range(t_adj4+100):
		if i<(t_adj4):
			ctrlData = np.radians(q_adj4[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_adj4[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	q_adj5 = RC.squeezingnarrowControl(4)
	t_adj5 = len(q_adj5)
	for i in range(t_adj5+500):
		if i<(t_adj5):
			ctrlData = np.radians(q_adj5[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_adj5[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	# Second half
	q_adj6 = RC.squeezingnarrowControl(5)

	t_5to6 = 400
	q_5to6 = np.array([np.linspace(q_adj5[-1][i], q_adj6[0][i], int(t_5to6/2)) for i in range(14)]).T
	for i in range(t_5to6):
		if i<(t_5to6/2):
			ctrlData = np.radians(q_5to6[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_5to6[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	
	t_adj6 = 10000
	for i in range(t_adj6):
		stepNUM = i % len(q_adj6)
		ctrlData = np.radians(q_adj6[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		#torso passed
		passed_torso = robot.holePosition()
		if(passed_torso)<=-0.015:
			q_passed_torso = q_adj6[stepNUM]
			break

	# back to normal locomotion
	q_finished = RC.squeezingnarrowControl(6)
	t_6toend = 400
	q_6toend= np.array([np.linspace(q_passed_torso[i], q_finished[0][i], int(t_6toend/2)) for i in range(14)]).T
	for i in range(t_6toend):
		if i<(t_6toend/2):
			ctrlData = np.radians(q_6toend[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_6toend[-1])
			robot.ctrlServo(ctrlData, timeStep)

	t_finished = len(q_finished)
	for i in range(t_finished+200):
		if i<(t_finished):
			ctrlData = np.radians(q_finished[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_finished[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	t_toContinue = 400
	q_toContinue = np.array([np.linspace(q_finished[-1][i], q_move[0][i], int(t_toContinue/2)) for i in range(14)]).T
	#print(q_transiton[0])
	for i in range(t_toContinue):
		if i<(t_toContinue/2):
			ctrlData = np.radians(q_toContinue[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_toContinue[-1])
			robot.ctrlServo(ctrlData, timeStep)

	t_continue = 2000
	for i in range(t_continue):
		stepNUM = i % len(q_move)
		ctrlData = np.radians(q_move[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	