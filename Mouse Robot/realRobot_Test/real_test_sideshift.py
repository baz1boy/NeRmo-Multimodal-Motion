from real_control import RobotDriver
from real_controller import robotController
import time
import numpy as np

# INIT_STEPS = 
if __name__ == '__main__':
	timeStep = 0.02  # Unit: s

	mouse = RobotDriver('192.168.2.84', 6666, timeStep)
	print("Robot Connected !!!")

	RC = robotController()
	step_num = RC.stepNum
	T = step_num*timeStep

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
	
	## lateral walk test:
	p0_side = np.array([[0.0325, -0.0060675, -0.05], [-0.0325, -0.0060675, -0.05], 
				 		[0.0325, -0.0060675, -0.05], [-0.0325, -0.0060675, -0.05]])
	side_d = -0.005
	side_h = 0.006
	q_sidemove = RC.sideControl(step_num, p0_side, side_d, side_h)

	t_phase1 = 200
	q_phase1 = np.array([np.linspace(q_init[i], q_sidemove[0][i], t_phase1) for i in range(14)]).T
	for i in range(t_phase1):
		if i<(t_phase1/2):
			start_time = time.time()
			ctrlData = q_phase1[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_phase1[-1]
			mouse.runStep(ctrlData, start_time)

	t_sidemove = len(q_sidemove)
	for i in range(1200):
		stepNUM = i % t_sidemove
		ctrlData = q_sidemove[stepNUM]
		mouse.runStep(ctrlData, start_time)


	
	
	
	mouse.shutdown()
		

