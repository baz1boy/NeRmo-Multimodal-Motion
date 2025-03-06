from matplotlib import pyplot as plt
from sim_control import simModel
from sim_controller import robotController
import numpy as np
from ahrs.filters import Madgwick
import transforms3d

INIT_STEPS = 2000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	robot = simModel("models_12DOF/robot_body_withhead.xml")
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
	q_phase = np.array([np.linspace(init_diff[i], q_init[i], t_init) for i in range(14)]).T
	for i in range(t_init):
		ctrlData = np.radians(q_phase[i])
		robot.ctrlServo(ctrlData, timeStep)
		
	acc = robot.acc
	gyro = robot.gyro
	# print(acc[-1])
	# print(gyro[-1])
	
	### for roll over
	# t1 = 200
	# for i in range(200):
	# 	q = q_phase[-1]
	# 	q[4] = -80
	# 	q[10] = -80
	# 	# q[7] = -80
	# 	# q[13] = -80
	# 	ctrlData = np.radians(q)
	# 	robot.ctrlServo(ctrlData, timeStep)
	
	# t2 = 500
	# for i in range(500):
	# 	q = q_phase[-1]
	# 	q[4] = 0
	# 	q[10] = 0
	# 	# q[7] = 0
	# 	# q[13] = 0
	# 	ctrlData = np.radians(q)
	# 	robot.ctrlServo(ctrlData, timeStep)

	## back on the ground
	t1 = 100
	for i in range(t1):
		q = q_phase[-1]
		q[4] = -80
		q[10] = -80
		#q[7] = -80
		#q[13] = -80
		ctrlData = np.radians(q)
		robot.ctrlServo(ctrlData, timeStep)

	t2 = 100
	for i in range(t2):
		q = q_phase[-1]
		q[4] = 0
		q[10] = 0
		#q[7] = 0
		#q[13] = 0
		ctrlData = np.radians(q)
		robot.ctrlServo(ctrlData, timeStep)

	t_stab = 1000
	for i in range(t_stab):
		ctrlData = np.radians(q_phase[-1])
		robot.ctrlServo(ctrlData, timeStep)
		
	madgwick = Madgwick()
	
	t_fall = t_init+t1+t2+t_stab
	q = np.array([1.0, 0.0, 0.0, 0.0])
	for i in range(t_fall):
		q = madgwick.updateIMU(q=q, gyr=gyro[i], acc=acc[i])
	
	euler_angle = transforms3d.euler.quat2euler(q, axes='sxyz')
	print(np.degrees(euler_angle))
	print(acc[-1])


	q_recover = RC.recoverControl(pleg, np.degrees(euler_angle), acc[-1])
	stepNum = len(q_recover)
	for i in range(stepNum+1000):
		if i < stepNum:
			ctrlData = np.radians(q_recover[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_recover[-1])
			robot.ctrlServo(ctrlData, timeStep)

	# plt.figure(1)
	# plt.plot([robot.torso_shoulder_x[t_fall], robot.torso_hip_x[t_fall]], [robot.torso_shoulder_y[t_fall], robot.torso_hip_y[t_fall]],
	# 		color='red', lw=3, marker='o', label='torso before')
	# plt.plot([robot.torso_shoulder_x[t_fall+150], robot.torso_hip_x[t_fall+150]], [robot.torso_shoulder_y[t_fall+150], robot.torso_hip_y[t_fall+150]],
	# 	  	color='blue', lw=3, marker='o', label='torso after')
	# plt.plot(robot.footend_fr_x[t_fall], robot.footend_fr_y[t_fall],
	# 		marker='o', color='green', label='footend before')
	# plt.plot(robot.footend_fr_x[t_fall+150], robot.footend_fr_y[t_fall+150],
	# 		marker='o', color='black', label='footend after')
	# plt.plot(robot.footend_hr_x[t_fall], robot.footend_hr_y[t_fall],
	# 		marker='o', color='green', label='footend before')
	# plt.plot(robot.footend_hr_x[t_fall+150], robot.footend_hr_y[t_fall+150],
	# 		marker='o', color='black', label='footend after')
	# plt.plot(robot.comx[t_fall], robot.comy[t_fall],
	# 		marker='o', color='red', label='com before')
	# plt.plot(robot.comx[t_fall+150], robot.comy[t_fall+150],
	# 		marker='o', color='blue', label='com after')
	
	# plt.title('Robot Dorsal Plane', fontsize=45)
	# plt.xlabel('x data', fontsize=35)
	# plt.ylabel('y data, initial facing', fontsize=35)
	# plt.xticks(fontsize=30)
	# plt.yticks(fontsize=30)
	# plt.axis('equal')
	# plt.legend(fontsize='large', loc='upper right', shadow=True, fancybox=True)

	# plt.show()