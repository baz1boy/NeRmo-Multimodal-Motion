from matplotlib import pyplot as plt
from sim_control import simModel
from sim_controller import robotController, q2p
import numpy as np

INIT_STEPS = 2000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	robot = simModel("models/robot_body_move.xml")
	RC = robotController()
	T = 2
	timeStep = 0.002

	init_diff = RC.init_qdiff  	# due to model setup, modeling data
	pleg = RC.init_pleg
	pbody = RC.init_pbody
	angle = RC.init_angle
	center = RC.init_center
	step_diff = RC.stepDiff
	#print(step_diff)

	## initialization
	q0 = np.array([0,0])
	q_init = RC.init_q
	q_init = np.hstack((q0,q_init))
	init_diff = np.hstack((q0,init_diff))
	
	t_init = 1000
	q_phase = np.array([np.linspace(init_diff[i], q_init[i], int(t_init/2)) for i in range(14)]).T
	for i in range(t_init):
		if i<(t_init/2):
			ctrlData = np.radians(q_phase[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_phase[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

    ## Start up
	h = 0.020
	y_s = -0.02445 # middle point between start and end point
	for i in range(4):
		pleg[i][1] = y_s
	q_1 = q_phase[-1]
	q_up = RC.heightCtrl(pleg, h)
	q_2 = q_up[-1]
	t_start = 1000
	q_start = np.array([np.linspace(q_1[i], q_2[i], int(t_start/2)) for i in range(14)]).T
	for i in range(t_start):
		if i<(t_start/2):
			ctrlData = np.radians(q_start[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_start[-1])
			robot.ctrlServo(ctrlData, timeStep)
	pleg_cur = q2p(q_start[-1])
	#print(pleg_cur)

	## Start Moving
	d = 0.006 
	h = 0.003
	pleg = pleg_cur
	for i in range(4):
		pleg[i][1] = pleg[i][1] + d/2

	q_left, q_right = RC.straightControl(pleg, d, h)
	n = len(q_left)				## step per cycle
	#print(q_left)

	### Start to Move
	t_straight = 6000
	h_top = []
	t_top = []
	for i in range(t_straight):
		q0 = q_left[(i + step_diff[0]) % n]
		q1 = q_right[(i + step_diff[1]) % n]
		q2 = q_left[(i + step_diff[2]) % n]
		q3 = q_right[(i + step_diff[3]) % n]
		q_head = np.array([0,0])
		q_straight = np.concatenate((q_head, q0, q1, q2, q3))
		ctrlData = np.radians(q_straight)
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		top = robot.spinePosition()
		#print(top)
		h_top.append(top*1000)
		t_top.append(i*0.002)

		# d_door = robot.squeezingPosition()
		# if(d_door)<=0:
		# 	print(i)
		# 	break

	plt.plot(t_top, h_top, lw='2.5')
	plt.title('robot height', fontsize=45)
	plt.xlabel('time(s)', fontsize=35)
	plt.ylabel('robot top(mm)', fontsize=35)
	plt.xticks(fontsize=30)
	plt.yticks(fontsize=30)
	#plt.legend(fontsize='large', loc='upper right', shadow=True, fancybox=True)
	plt.show()

	

	


	