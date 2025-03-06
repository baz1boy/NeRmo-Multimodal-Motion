from real_control import RobotDriver
from real_controller import robotController, q2p, p2q
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
	
	## recover test:
	euler_angle = [0, 70, 0]
	acc = [9, 0, 0]
	# euler_angle = [0, 10, 0]
	# acc = [0, 0, -9]
	q_recover = RC.recoverControl(pleg, euler_angle, acc)
	t_recover = len(q_recover)
	for i in range(t_recover+100):
		if i < t_recover:
			start_time = time.time()
			ctrlData = q_recover[i]
			mouse.runStep(ctrlData, timeStep)
		else:
			start_time = time.time()
			ctrlData = q_recover[-1]
			mouse.runStep(ctrlData, timeStep)
	
	
	mouse.shutdown()
		

