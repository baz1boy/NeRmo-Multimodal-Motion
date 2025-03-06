from matplotlib import pyplot as plt
from sim_control import simModel
from sim_controller import robotController, q2p, p2q
import numpy as np


INIT_STEPS = 2000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	robot = simModel("models/robot_body.xml")
	RC = robotController()
	T = 2
	timeStep = 0.002

	init_diff = RC.init_qdiff  	# due to model setup, modeling data
	pleg = RC.init_pleg
	pbody = RC.init_pbody
	angle = RC.init_angle
	center = RC.init_center
	#print(pbody)

	## initialization
	q0 = np.array([0,0])
	q_init = RC.init_q
	q_init = np.hstack((q0,q_init))
	init_diff = np.hstack((q0,init_diff))

	t_init = 500
	q_start = np.array([np.linspace(init_diff[i], q_init[i], t_init) for i in range(14)]).T
	for i in range(t_init):
		ctrlData = np.radians(q_start[i])
		robot.ctrlServo(ctrlData, timeStep)
		
    ## Jump Motion
	## 1.Ab/Ad Squat Jump
	# q_jump1 = RC.jumpControl(pleg, 0)
	# t_jump1 = len(q_jump1)
	# for i in range(t_jump1+1000):
	# 	if i<t_jump1:
	# 		ctrlData = np.radians(q_jump1[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 		if i>600 and i<=640:
	# 			foot_force = robot.getForce()
	# 			print(foot_force)
	# 	else:
	# 		ctrlData = np.radians(q_jump1[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# pleg_cur = q2p(q_jump1[-1])

	# F_total = np.zeros_like(robot.F_fl)
	# for i in range(len(robot.F_fl)):
	# 	F_total[i] = robot.F_fl[i] + robot.F_fr[i] + robot.F_hl[i] + robot.F_hr[i]
	# print(F_total)

	# plt.figure(1)
	# x = range(1, len(F_total) + 1)
	# plt.plot(x, F_total, marker='o')

	# plt.title('Force applied by four feet', fontsize=45)
	# plt.xlabel('Index', fontsize=35)
	# plt.ylabel('Force (N)', fontsize=35)
	# plt.xticks(fontsize=30)
	# plt.yticks(fontsize=30)

	# plt.grid(True)
	# plt.show()

	## 2.Hip/Knee Squat Jump
	# q_jump2 = RC.jumpControl(pleg, 1)
	# t_jump2 = len(q_jump2)
	# for i in range(t_jump2+1000):
	# 	if i<t_jump2:
	# 		ctrlData = np.radians(q_jump2[i])
	# 		robot.ctrlServo(ctrlData, timeStep)
	# 		if i>600 and i<=640:
	# 			foot_force = robot.getForce()
	# 			print(foot_force)
	# 	else:
	# 		ctrlData = np.radians(q_jump2[-1])
	# 		robot.ctrlServo(ctrlData, timeStep)
	
	# F_total = np.zeros_like(robot.F_fl)
	# for i in range(len(robot.F_fl)):
	# 	F_total[i] = robot.F_fl[i] + robot.F_fr[i] + robot.F_hl[i] + robot.F_hr[i]
	# print(F_total)

	# plt.figure(1)
	# x = range(1, len(F_total) + 1)
	# plt.plot(x, F_total, marker='o')

	# plt.title('Force applied by four feet', fontsize=45)
	# plt.xlabel('Index', fontsize=35)
	# plt.ylabel('Force (N)', fontsize=35)
	# plt.xticks(fontsize=30)
	# plt.yticks(fontsize=30)

	# plt.grid(True)
	# plt.show()

	## 3.Hybrid Squat Jump
	q_jump3 = RC.jumpControl(pleg, 2)
	t_jump3 = len(q_jump3)
	print(t_jump3)
	for i in range(t_jump3+1000):
		if i<t_jump3:
			ctrlData = np.radians(q_jump3[i])
			robot.ctrlServo(ctrlData, timeStep)
			if i>600 and i<=640:
				foot_force = robot.getForce()
				h_dynamic = robot.getCenter()[2]
				#print(h_dynamic)
		else:
			ctrlData = np.radians(q_jump3[-1])
			robot.ctrlServo(ctrlData, timeStep)
	# h_static = robot.getCenter()[2]
	# print('static:', h_static)

	F_total = np.zeros_like(robot.F_fl)
	for i in range(len(robot.F_fl)):
		F_total[i] = robot.F_fl[i] + robot.F_fr[i] + robot.F_hl[i] + robot.F_hr[i]
	print(F_total)

	plt.figure(1)
	x = range(1, len(F_total) + 1)
	plt.plot(x, F_total, marker='o')

	plt.title('Force applied by four feet', fontsize=45)
	plt.xlabel('Index', fontsize=35)
	plt.ylabel('Force (N)', fontsize=35)
	plt.xticks(fontsize=30)
	plt.yticks(fontsize=30)

	plt.grid(True)
	plt.show()