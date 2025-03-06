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
	t_move = 400
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

	# bow head:
	t_tran2squ = 100
	q_squ1 = RC.squeezingControl(0)
	q_tran2squ = np.array([np.linspace(q_reach[i], q_squ1[0][i], int(t_tran2squ)) for i in range(14)]).T
	for i in range(t_tran2squ):
		if i<(t_tran2squ+100):
			start_time = time.time()
			ctrlData = q_tran2squ[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_tran2squ[-1]
			mouse.runStep(ctrlData, timeStep)

	t_squ1 = 700
	for i in range(t_squ1):
		start_time = time.time()
		stepNUM = i % len(q_squ1)
		ctrlData = q_squ1[stepNUM]
		mouse.runStep(ctrlData, timeStep)

		# head passed:
		q_passed_head = q_squ1[stepNUM]
		step_passed_head = i

	# crawling move:
	q_squ2 = RC.squeezingControl(1)

	t_1to2 = 100
	q_1to2 = np.array([np.linspace(q_passed_head[i], q_squ2[0][i], int(t_1to2)) for i in range(14)]).T
	for i in range(t_1to2+100):
		if i<(t_1to2):
			start_time = time.time()
			ctrlData = q_1to2[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_1to2[-1]
			mouse.runStep(ctrlData, timeStep)

	t_squ2 = 6800
	for i in range(t_squ2):
		stepNUM = i % len(q_squ2)
		start_time = time.time()
		ctrlData = q_squ2[stepNUM]
		mouse.runStep(ctrlData, timeStep)

		# torso passed:
		q_passed_torso = q_squ2[stepNUM]
		step_passed_torso = i

	# motion traj adjust
	q_squ3 = RC.squeezingControl(2)
	q_passed_torso_pre = q_passed_torso.copy()
	q_passed_torso_pre[-12:] = [20,90,0] * 4

	t_2to3_pre = 100
	q_2to3_pre = np.array([np.linspace(q_passed_torso[i], q_passed_torso_pre[i], int(t_2to3_pre)) for i in range(14)]).T
	for i in range(t_2to3_pre+100):
		if i<(t_2to3_pre):
			start_time = time.time()
			ctrlData = q_2to3_pre[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_2to3_pre[-1]
			mouse.runStep(ctrlData, timeStep)

	t_2to3 = 100
	q_2to3 = np.array([np.linspace(q_2to3_pre[-1][i], q_squ3[0][i], int(t_2to3)) for i in range(14)]).T
	for i in range(t_2to3+1000):
		if i<(t_2to3):
			start_time = time.time()
			ctrlData = q_2to3[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_2to3[-1]
			mouse.runStep(ctrlData, timeStep)
	
	t_squ3 = 1000
	for i in range(t_squ3):
		stepNUM = i % len(q_squ3)
		start_time = time.time()
		ctrlData = q_squ3[stepNUM]
		mouse.runStep(ctrlData, timeStep)










	
	
	
	mouse.shutdown()
		

