from real_control import RobotDriver
from real_controller import robotController
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
	t_init = 400
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
	
	# # turning test:
	# deg = -90  # here change the turning angle		123
	# q_turn_leg = RC.turnRotation(deg, pleg, angle, center)
	# t_turn = len(q_turn_leg)
	# q_head = np.zeros((t_turn, 2))
	# q_turn = np.hstack((q_head, q_turn_leg))
	# for i in range(t_turn + 200):
	# 	if i < t_turn:
	# 		ctrlData = q_turn[i]
	# 		mouse.runStep(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = q_turn[-1]
	# 		mouse.runStep(ctrlData, timeStep)

	# deg = -170  # here change the turning angle		-170
	# q_turn_leg = RC.turnRotation(deg, pleg, angle, center)
	# t_turn = len(q_turn_leg)
	# q_head = np.zeros((t_turn, 2))
	# q_turn = np.hstack((q_head, q_turn_leg))
	# for i in range(t_turn + 200):
	# 	if i < t_turn:
	# 		ctrlData = q_turn[i]
	# 		mouse.runStep(ctrlData, timeStep)
	# 	else:
	# 		ctrlData = q_turn[-1]
	# 		mouse.runStep(ctrlData, timeStep)
	
	
	mouse.shutdown()
		

