from real_control import RobotDriver
from real_controller import robotController
import time
import numpy as np

# INIT_STEPS = 
if __name__ == '__main__':
	timeStep = 0.02  # Unit: s

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
	
	### move test:
	p0 = np.array([[0.0325, 0.0039325, -0.050], [-0.0325, 0.0039325, -0.050], 
				 	[0.0325, 0.0039325, -0.050], [-0.0325, 0.0039325, -0.050]])
	p1 = np.array([[0.0325, -0.0160675, -0.050], [-0.0325, -0.0160675, -0.050], 
					[0.0325, -0.0160675, -0.050], [-0.0325, -0.0160675, -0.050]])		
	h_lift = 0.01
	q_move = RC.moveControl(p0, p1, h_lift)
	n = len(q_move)
	t_transtion = 200
	q_transiton = np.array([np.linspace(q_initialization[-1][i], q_move[0][i], int(t_transtion/2)) for i in range(14)]).T
	#print(q_transiton[0])
	for i in range(t_transtion):
		if i<(t_transtion/2):
			ctrlData = q_transiton[i]
			mouse.runStep(ctrlData, start_time)
		else:
			ctrlData = q_transiton[-1]
			mouse.runStep(ctrlData, start_time)
	
	# move
	t_move = 2000
	for i in range(t_move):
		stepNUM = i % n
		ctrlData = q_move[stepNUM]
		mouse.runStep(ctrlData, start_time)
	
	
	mouse.shutdown()
		

