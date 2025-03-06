from matplotlib import pyplot as plt
from sim_control import simModel
from sim_controller import robotController, q2p, p2q
import numpy as np
import pandas as pd

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
	#print(stepdiff)

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
	
	### Trajectory Point
	pick = 5
	s = 0.005
	if pick==1:
		p0 = np.array([[0.035, -0.02145, -0.020], [-0.035, -0.02145, -0.020], [0.035, -0.02145, -0.020], [-0.035, -0.02145, -0.020],])
		p1 = np.array([[0.035, -0.02745, -0.020], [-0.035, -0.02745, -0.020], [0.035, -0.02745, -0.020], [-0.035, -0.02745, -0.020],])
	elif pick==2:
		p0 = np.array([[0.0375, -0.02145, -0.020], [-0.0375, -0.02145, -0.020], [0.0375, -0.02145, -0.020], [-0.0375, -0.02145, -0.020],])
		p1 = np.array([[0.0375, -0.02745, -0.020], [-0.0375, -0.02745, -0.020], [0.0375, -0.02745, -0.020], [-0.0375, -0.02745, -0.020],])
	elif pick==3:
		p0 = np.array([[0.04, -0.02145, -0.020], [-0.04, -0.02145, -0.020], [0.04, -0.02145, -0.020], [-0.04, -0.02145, -0.020],])
		p1 = np.array([[0.04, -0.02745, -0.020], [-0.04, -0.02745, -0.020], [0.04, -0.02745, -0.020], [-0.04, -0.02745, -0.020],])
	elif pick==4:
		p0 = np.array([[0.0375, -0.02145+s, -0.020], [-0.0375, -0.02145+s, -0.020], 
				 		[0.0375, -0.02145+s, -0.020], [-0.0375, -0.02145+s, -0.020],])
		p1 = np.array([[0.0375, -0.02745-s, -0.020], [-0.0375, -0.02745-s, -0.020], 
				 		[0.0375, -0.02745-s, -0.020], [-0.0375, -0.02745-s, -0.020],])
	elif pick==5: # leaning posture
		p = np.array([[0.0325, -0.024, -0.0278128], [-0.0325, -0.024, -0.0278128], 
				 		[0.0325, -0.024, -0.0278128], [-0.0325, -0.024, -0.0278128],])		


    ## Start up
	# pm = (p0+p1)/2		#lower posture
	pm = p		#leaning posture
	#print(pm)

	qm = p2q(pm)
	q_head = np.array([0,0])
	q_start = np.hstack((q_head, qm))

	t_startup = 1000
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

	# pitch roll yaw move:
	lean = np.array([5.628,0,0])
	q_pry,cur_pleg,cur_angle = RC.pryStance(pleg_cur, lean, angle, center)
	num = len(q_pry)
	q_head = np.array([0,0])
	for i in range(1000):
		if i < num:
			q_pitch = np.hstack((q_head, q_pry[i]))
			ctrlData = np.radians(q_pitch)
		else:
			q_pitch = np.hstack((q_head, q_pry[-1]))
			ctrlData = np.radians(q_pitch)
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
	#print(cur_angle)
	#print(cur_pleg)

	# move_distance = 0.006+s*2		#lower posture stride length
	move_distance = 0.003		#leaning posture stride length
	q_leanMove = RC.leanControl(cur_pleg, cur_angle, center, move_distance)
	n = len(q_leanMove)
	#print(q_leanMove)

	t_ready = 500
	q_ready = np.array([np.linspace(q_pitch[i], q_leanMove[0][i], int(t_ready/2)) for i in range(14)]).T
	for i in range(t_ready):
		if i<(t_ready/2):
			ctrlData = np.radians(q_ready[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_ready[-1])
			robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()

	h_top = []
	t_top = []
	for i in range(5000):
		stepNUM = i % n
		ctrlData = np.radians(q_leanMove[stepNUM])
		robot.ctrlServo(ctrlData, timeStep)
		robot.ctrlViewer()
		top = robot.topPosition()
		h_top.append(top*1000)
		t_top.append(i*0.002)

	data = pd.DataFrame({'Time (s)': t_top, 'Height (mm)': h_top})
	data.to_csv('robot_data_l3.csv', index=False)
	plt.rcParams['font.family'] = 'Times New Roman'

	plt.figure(figsize=(10, 6))
	plt.plot(t_top, h_top, lw=2, color='b', label='Height Curve')
	plt.title('Robot Top Position Over Time', fontsize=16)  #
	plt.xlabel('Time (s)', fontsize=14)  # 
	plt.ylabel('Height (mm)', fontsize=14)

	plt.ylim(min(h_top) - 2, max(h_top) + 2)

	plt.grid(True, linestyle='--', alpha=0.6) 
	plt.xticks(fontsize=12) 
	plt.yticks(fontsize=12)

	plt.legend(fontsize=12, loc='upper right', shadow=True, fancybox=True)

	plt.tight_layout()
	plt.savefig('robot_height_plot.pdf', format='pdf', bbox_inches='tight')
	plt.show()
	

	


	