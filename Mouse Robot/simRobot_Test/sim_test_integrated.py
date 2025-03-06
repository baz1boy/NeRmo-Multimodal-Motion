from matplotlib import pyplot as plt
from sim_control_multi import simModel
from sim_controller import robotController, q2p, p2q
import numpy as np
from ahrs.filters import Madgwick
import transforms3d

INIT_STEPS = 2000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	robot = simModel("models/robot_body_integrated.xml")
	RC = robotController()
	timeStep = 0.002
	step_num = RC.stepNum
	T = step_num*timeStep

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
	q_phase0 = np.array([np.linspace(init_diff[i], q_init[i], int(t_init/2)) for i in range(14)]).T
	for i in range(t_init):
		if i<(t_init/2):
			ctrlData = np.radians(q_phase0[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_phase0[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	init_quat = robot.getQuat()
	#print(init_quat)

	# deg = -180  # here change the turning angle
	# q_turn = RC.turnRotation(deg, pleg, angle, center)
	# t_turn = 10000
	# q_head = np.zeros((len(q_turn), 2))
	# q_turn = np.hstack((q_head, q_turn))
	# for i in range(t_turn):
	# 	ctrlData = np.radians(q_turn[i])
	# 	robot.ctrlServo(ctrlData, timeStep)
	# 	cur_quat = robot.getQuat()
	# 	#print(cur_quat)
	# 	rotation_deg = RC.turnAngle(init_quat, cur_quat)
	# 	if(rotation_deg<=-90):
	# 		q_turned = q_turn[i]
	# 		print(rotation_deg)
	# 		break

	deg = -5  # here change the turning angle
	q_turn = RC.turnRotation(deg, pleg, angle, center)
	t_turn = len(q_turn)
	q_head = np.zeros((len(q_turn), 2))
	q_turn = np.hstack((q_head, q_turn))
	for j in range(1000):
		for i in range(t_turn):
			ctrlData = np.radians(q_turn[i])
			robot.ctrlServo(ctrlData, timeStep)
			robot.ctrlViewer()
		cur_quat = robot.getQuat()
		#print(cur_quat)
		rotation_deg = RC.turnAngle(init_quat, cur_quat)
		if(rotation_deg<=-90):
			q_turned = q_turn[i]
			print(rotation_deg)
			break
			
	
	# Side shift
	p0_side = np.array([[0.0325, 0.0060675, -0.05], [-0.0325, 0.0060675, -0.05], 
				 		[0.0325, 0.0060675, -0.05], [-0.0325, 0.0060675, -0.05]])
	side_d = -0.002
	side_h = 0.002
	q_sidemove = RC.sideControl(step_num, p0_side, side_d, side_h)

	t_phase1 = 500
	q_phase1 = np.array([np.linspace(q_init[i], q_sidemove[0][i], t_phase1) for i in range(14)]).T
	for i in range(t_phase1):
		if i<(t_phase1/2):
			ctrlData = np.radians(q_phase1[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_phase1[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	t_sidemove = len(q_sidemove)
	for i in range(10000):
		stepNUM = i % t_sidemove
		ctrlData = np.radians(q_sidemove[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		facepos = robot.facePosition()
		if facepos>= -0.015:
			q_facinghole = q_sidemove[stepNUM]
			break
	
	### Trajectory Point
	p0 = np.array([[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05], 
				 	[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05]])
	p1 = np.array([[0.0325, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05], 
					[0.0325, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05]])		
	pm = (p0+p1)/2
	qm = p2q(pm)
	q_head = np.array([0,0])
	q_start = np.hstack((q_head, qm))

	## Start Moving
	h_lift = 0.005
	q_move = RC.moveControl(p0, p1, h_lift)
	
	t_transtion = 500
	q_transiton = np.array([np.linspace(q_facinghole[i], q_move[0][i], int(t_transtion/2)) for i in range(14)]).T
	for i in range(t_transtion):
		if i<(t_transtion/2):
			ctrlData = np.radians(q_transiton[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_transiton[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	### Squeezing Test
	t_move = 5000
	for i in range(t_move):
		stepNUM = i % len(q_move)
		ctrlData = np.radians(q_move[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

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
	