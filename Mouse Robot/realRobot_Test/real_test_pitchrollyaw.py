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
	


	# pitch roll yaw move:
	move = np.array([10,0,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(pleg, move, angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([-10,0,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([-10,0,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(pleg, move, angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([10,0,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,12,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,-12,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,-12,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,12,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,0,7])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,0,-7])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,0,-7])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([0,0,7])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([-5,12,-7])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([5,-12,7])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([8,-20,8])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)
	print(cur_angle)

	move = np.array([-8,20,-8])
	q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	num = len(q_pry)
	for i in range(60):
		start_time = time.time()
		if i < num:
			ctrlData = q_pry[i]
		else:
			ctrlData = q_pry[-1]

		mouse.runStep(ctrlData, start_time)


	
	
	
	mouse.shutdown()
		

