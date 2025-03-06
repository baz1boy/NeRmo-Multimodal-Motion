from real_control import RobotDriver
from real_controller import robotController, q2p, p2q
import time
import numpy as np

# INIT_STEPS = 
if __name__ == '__main__':
	timeStep = 0.002  # Unit: s

	mouse = RobotDriver('192.168.2.84', 6666, timeStep)
	print("Robot Connected !!!")

	T = 2  # frequency = 0.5 
	RC = robotController()

	init_diff = RC.init_qdiff  	# due to model setup, modeling data
	pleg = RC.init_pleg
	pbody = RC.init_pbody
	angle = RC.init_angle
	center = RC.init_center

	print("Controller initiated !!!")

	## Mouse Initialization
	q0 = np.array([0,0])
	q_init = RC.init_q
	q_init = np.hstack((q0,q_init))
	init_diff = np.hstack((q0,init_diff))
	
	print("Step 1 -->")
	t_init = 100
	for i in range(t_init):
		start_time = time.time()
		ctrlData = q_init
		mouse.runStep(ctrlData, start_time)

	print("Step 2 -->")
	q_initialization = np.array([np.linspace(q_init[i], q_init[i], int(t_init/2)) for i in range(14)]).T
	for i in range(t_init):
		if i<(t_init/2):
			start_time = time.time()
			ctrlData = q_initialization[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_initialization[-1]
			mouse.runStep(ctrlData, start_time)
		
	print("Robot initialized !!!")
	
	"""
	Squeezing Test:
	"""
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
	t_startup = 200
	q_startup = np.array([np.linspace(q_initialization[-1][i], q_start[i], int(t_startup/2)) for i in range(14)]).T
	for i in range(t_startup):
		if i<(t_startup/2):
			start_time = time.time()
			ctrlData = q_startup[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_startup[-1]
			mouse.runStep(ctrlData, start_time)
	pleg_cur = q2p(q_startup[-1])

	## Start Moving
	h_lift = 0.005
	q_move = RC.moveControl(p0, p1, h_lift)
	n = len(q_move)

	t_transtion = 200
	q_transiton = np.array([np.linspace(q_startup[-1][i], q_move[0][i], int(t_transtion/2)) for i in range(14)]).T
	for i in range(t_transtion):
		if i<(t_transtion/2):
			start_time = time.time()
			ctrlData = q_transiton[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_transiton[-1]
			mouse.runStep(ctrlData, timeStep)
	
	### Squeezing Test
	t_move = 200
	q_reach = np.zeros_like(q_transiton[-1])
	for i in range(t_move):
		stepNUM = i % n
		start_time = time.time()
		ctrlData = q_move[stepNUM]
		mouse.runStep(ctrlData, timeStep)
		# hole detected:
		# hole detected:
		# hole detected:
		q_reach = q_move[stepNUM]

	t_pause = 50
	for i in range(t_pause):
		start_time = time.time()
		ctrlData = q_reach
		mouse.runStep(ctrlData, timeStep)

	"""
	Through a narrow hole
	"""
	# adjust posture
	t_adj1 = 100
	q_adj1 = RC.squeezingnarrowControl(0)

	q_nar1 = np.array([np.linspace(q_reach[i], q_adj1[i], int(t_adj1)) for i in range(14)]).T
	for i in range(t_adj1):
		if i<(t_adj1+100):
			start_time = time.time()
			ctrlData = q_nar1[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_nar1[-1]
			mouse.runStep(ctrlData, timeStep)

	q_adj2 = RC.squeezingnarrowControl(1)
	t_adj2 = len(q_adj2)
	for i in range(t_adj2+200):
		if i<(t_adj2):
			start_time = time.time()
			ctrlData = q_adj2[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_adj2[-1]
			mouse.runStep(ctrlData, timeStep)

	# First half
	q_adj3 = RC.squeezingnarrowControl(2)
	t_2to3 = 100
	q_2to3 = np.array([np.linspace(q_adj2[-1][i], q_adj3[0][i], int(t_2to3)) for i in range(14)]).T
	for i in range(t_2to3+100):
		if i<(t_2to3):
			start_time = time.time()
			ctrlData = q_2to3[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_2to3[-1]
			mouse.runStep(ctrlData, timeStep)
	
	t_adj3 = 2000
	for i in range(t_adj3):
		stepNUM = i % len(q_adj3)
		start_time = time.time()
		ctrlData = q_adj3[stepNUM]
		mouse.runStep(ctrlData, timeStep)
		#half body passed
		#half body passed
		#half body passed
		q_half_passed = q_adj3[stepNUM]
			
	# transitional phase, adjust pos
	q_adj4 = RC.squeezingnarrowControl(3)
	t_3to4 = 100
	q_3to4 = np.array([np.linspace(q_half_passed[i], q_adj4[0][i], int(t_3to4)) for i in range(14)]).T
	for i in range(t_3to4+100):
		if i<(t_3to4):
			start_time = time.time()
			ctrlData = q_3to4[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_3to4[-1]
			mouse.runStep(ctrlData, timeStep)
	
	t_adj4 = len(q_adj4)
	for i in range(t_adj4+100):
		if i<(t_adj4):
			start_time = time.time()
			ctrlData = q_adj4[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_adj4[-1]
			mouse.runStep(ctrlData, timeStep)

	q_adj5 = RC.squeezingnarrowControl(4)
	t_adj5 = len(q_adj5)
	for i in range(t_adj5+100):
		if i<(t_adj5):
			start_time = time.time()
			ctrlData = q_adj5[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_adj5[-1]
			mouse.runStep(ctrlData, timeStep)

	# Second half
	q_adj6 = RC.squeezingnarrowControl(5)

	t_5to6 = 100
	q_5to6 = np.array([np.linspace(q_adj5[-1][i], q_adj6[0][i], int(t_5to6)) for i in range(14)]).T
	for i in range(t_5to6+100):
		if i<(t_5to6):
			start_time = time.time()
			ctrlData = q_5to6[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_5to6[-1]
			mouse.runStep(ctrlData, timeStep)
	
	t_adj6 = 2400
	for i in range(t_adj6):
		stepNUM = i % len(q_adj6)
		start_time = time.time()
		ctrlData = q_adj6[stepNUM]
		mouse.runStep(ctrlData, timeStep)
		#torso passed
		#torso passed
		#torso passed
		q_passed_torso = q_adj6[stepNUM]

	# back to normal locomotion
	q_finished = RC.squeezingnarrowControl(6)
	t_6toend = 100
	q_6toend= np.array([np.linspace(q_passed_torso[i], q_finished[0][i], int(t_6toend)) for i in range(14)]).T
	for i in range(t_6toend+100):
		if i<(t_6toend):
			start_time = time.time()
			ctrlData = q_6toend[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_6toend[-1]
			mouse.runStep(ctrlData, timeStep)

	t_finished = len(q_finished)
	for i in range(t_finished+100):
		if i<(t_finished):
			start_time = time.time()
			ctrlData = q_finished[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_finished[-1]
			mouse.runStep(ctrlData, timeStep)

	t_toContinue = 100
	q_toContinue = np.array([np.linspace(q_finished[-1][i], q_move[0][i], int(t_toContinue)) for i in range(14)]).T
	for i in range(t_toContinue+100):
		if i<(t_toContinue):
			start_time = time.time()
			ctrlData = q_toContinue[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_toContinue[-1]
			mouse.runStep(ctrlData, timeStep)

	t_continue = 1000
	for i in range(t_continue):
		stepNUM = i % len(q_move)
		start_time = time.time()
		ctrlData = q_move[stepNUM]
		mouse.runStep(ctrlData, timeStep)


	
	mouse.shutdown()
		

