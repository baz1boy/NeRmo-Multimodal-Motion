from matplotlib import pyplot as plt
from sim_control import simModel
from sim_controller import robotController
import numpy as np

INIT_STEPS = 2000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	robot = simModel("models_12DOF/robot_body.xml")
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
	q_init = RC.init_q
	q_phase = np.array([np.linspace(init_diff[i], q_init[i], 1000) for i in range(12)]).T
	for i in range(1000):
		ctrlData = np.radians(q_phase[i])
		robot.ctrlServo(ctrlData, timeStep)

	## pitch roll yaw move:
	# move = np.array([10,0,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(pleg, move, angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([-10,0,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([-10,0,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(pleg, move, angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([10,0,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,12,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,-12,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,-12,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,12,0])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,0,7])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,0,-7])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,0,-7])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([0,0,7])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([-5,12,-7])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([5,-12,7])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([8,-20,8])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)

	# move = np.array([-8,20,-8])
	# q_pry,cur_pleg,cur_angle = RC.pryStance(cur_pleg, move, cur_angle, center)
	# num = len(q_pry)
	# for i in range(200):
	# 	if i < num:
	# 		ctrlData = np.radians(q_pry[i])
	# 	else:
	# 		ctrlData = np.radians(q_pry[-1])

	# 	robot.ctrlServo(ctrlData, timeStep)
	# print(cur_angle)
	

	## turning test:
	deg = -45  # here change the turning angle
	q_turn = RC.turnRotation(deg, pleg, angle, center)
	stepNum = len(q_turn)
	for i in range(stepNum + 500):
		if i < stepNum:
			ctrlData = np.radians(q_turn[i])
			robot.ctrlServo(ctrlData, timeStep)
		else:
			ctrlData = np.radians(q_turn[-1])
			robot.ctrlServo(ctrlData, timeStep)


	plt.rcParams.update(
		{
			"font.family": "serif",
			"font.serif": "Times New Roman"
		}
	)

	# 
	x_min, x_max = -0.02, 0.02  
	y_min, y_max = -0.08, 0.08
	plt.figure(1)
	plt.plot(robot.torso_center_x[1000:], robot.torso_center_y[1000:], color=(1, 0.6, 0.1), lw=1, label='torso center offset')
	plt.plot([robot.torso_shoulder_x[1000], robot.torso_hip_x[1000]], [robot.torso_shoulder_y[1000], robot.torso_hip_y[1000]],
			color='red', lw=3, marker='o', label='torso before')
	plt.plot(robot.torso_center_x[1000], robot.torso_center_y[1000], marker='o', color='black')
	plt.plot([robot.torso_shoulder_x[-1], robot.torso_hip_x[-1]], [robot.torso_shoulder_y[-1], robot.torso_hip_y[-1]],
			color='blue', lw=3, marker='o', label='torso after')
	plt.plot(robot.torso_center_x[-1], robot.torso_center_y[-1], marker='o', color='black')
	plt.title('Robot Dorsal Plane')
	plt.xlabel('x data')
	plt.ylabel('y data (robot initial facing)')
	plt.xticks()
	plt.yticks()
	plt.axis('equal')
	plt.xlim(x_min, x_max)  
	plt.ylim(y_min, y_max)  
	plt.legend(fontsize='large', loc='upper right', shadow=True, fancybox=True)
	#plt.savefig("robot_dorsal_plane.pdf", format='pdf', bbox_inches='tight')

	x_min, x_max = -0.008, 0.008  
	y_min, y_max = -0.004, 0.004 
	plt.figure(2)
	plt.plot(robot.torso_center_x[1000:], robot.torso_center_y[1000:], color=(1, 0.6, 0.1), lw=2.5, label='torso center trajectory')
	plt.scatter(robot.torso_center_x[1000], robot.torso_center_y[1000], s=120, c='red', label='torso center before', zorder=2)
	plt.scatter(robot.torso_center_x[-1], robot.torso_center_y[-1], s=120, c='blue', label='torso center after', zorder=2)
	plt.title('Robot Torso Center')
	plt.xlabel('x data')
	plt.ylabel('y data (robot initial facing)')
	plt.xticks()
	plt.yticks()
	plt.axis('equal')
	plt.xlim(x_min, x_max)  
	plt.ylim(y_min, y_max)  
	plt.legend(loc='upper right', shadow=True, fancybox=True)
	#plt.savefig("robot_torso_center.pdf", format='pdf', bbox_inches='tight')
	plt.show()


	# print(robot.torso_center_y[999])
	# print(robot.torso_center_x[1000], robot.torso_center_x[-1])
	# print(robot.torso_center_y[1000], robot.torso_center_y[-1])
	

	#calculate the angle between lines
	u = np.array([robot.torso_shoulder_x[1000]-robot.torso_hip_x[1000], 
				robot.torso_shoulder_y[1000]-robot.torso_hip_y[1000]])
	v = np.array([robot.torso_shoulder_x[-1]-robot.torso_hip_x[-1], 
				robot.torso_shoulder_y[-1]-robot.torso_hip_y[-1]])
	
	dot_product = np.dot(u,v)
	norm_u = np.linalg.norm(u)
	norm_v = np.linalg.norm(v)

	cos_uv = dot_product/(norm_u*norm_v)
	angle_uv = np.arccos(cos_uv)
	angle_uv_deg = np.degrees(angle_uv)
	print(f"Acutual turning angle: {angle_uv_deg}")

	# calculate the displacement between two torso center points
	x1, y1 = robot.torso_center_x[1000], robot.torso_center_y[1000] 
	x2, y2 = robot.torso_center_x[-1], robot.torso_center_y[-1]     
	distance = ((x2 - x1)**2 + (y2 - y1)**2)**0.5

	print(f"Distance between the two points: {distance}")



	

	

