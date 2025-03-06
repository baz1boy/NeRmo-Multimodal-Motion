import numpy as np
from joint_position import compute_joints_position
from controller import LegController
from matplotlib import pyplot as plt
from matplotlib import animation

# from q_angle_calculation import Trajectory
# s = [-0.044, -0.060]
# gc = Trajectory()
# q1,q2,step= gc.q_values(s, 1, 0.002) 

# print("Calculated q1 values:", q1)
# print("Calculated q2 values:", q2)
# print("step", step)


INIT_STEPS = 10
SIM_STEPS = 200

if __name__ == '__main__':
	init_pos = [-0.0396388, -0.05]
	T = 2
	time_step = 0.01
	theController = LegController(init_pos, T, time_step)

	# print(theController.z_list)

	points_list = []
	q_list = []
	jointC = []
	jointD = []
	jointE = []
	jointF = []
	jointG = []
	jointH = []
	
	for i in range(SIM_STEPS):
		
		cur_q = theController.runCtrl(i,3)
		cur_I = theController.pos_foot_end()
		cur_jointC, _, _, _, _, _ = compute_joints_position(cur_q[0],cur_q[1])
		_, cur_jointD, _, _, _, _ = compute_joints_position(cur_q[0],cur_q[1])
		_, _, cur_jointE, _, _, _ = compute_joints_position(cur_q[0],cur_q[1])
		_, _, _, cur_jointF, _, _ = compute_joints_position(cur_q[0],cur_q[1])
		_, _, _, _, cur_jointG, _ = compute_joints_position(cur_q[0],cur_q[1])
		_, _, _, _, _, cur_jointH = compute_joints_position(cur_q[0],cur_q[1])
		q_list.append(cur_q)
		points_list.append(cur_I)
		jointC.append(cur_jointC)
		jointD.append(cur_jointD)
		jointE.append(cur_jointE)
		jointF.append(cur_jointF)
		jointG.append(cur_jointG)
		jointH.append(cur_jointH)
		

	# print("I_position", jointH)
	# print(len(jointH))
	# print(len(points_list))

	fig, ax = plt.subplots(figsize=(8, 6))
	
	point_num = len(points_list)
	
	xA = [0] * point_num
	yA = [0] * point_num
	xB = [0] * point_num
	yB = [0.0128] * point_num
	xC, yC = zip(*jointC)
	xD, yD = zip(*jointD)
	xE, yE = zip(*jointE)
	xF, yF = zip(*jointF)
	xG, yG = zip(*jointG)
	xH, yH = zip(*jointH)
	x8, y8 = zip(*points_list)
	ax.plot(x8, y8, lw=2, label='Trajectory')
	ax.set_aspect(1)
	ax.set_xlim(-0.1, 0.05)  
	ax.set_ylim(-0.1, 0.06)
	ax.set_xlabel("Y data")
	ax.set_ylabel("Z data")
	ax.grid(True)	
	
	lines = plt.plot([], [], 'k-',  # AB
			[], [], 'r-',			# BC
			[], [], 'r-',			# AE
			[], [], 'r-',			# AF
			[], [], 'r-',			# EF
			[], [], 'b-',			# CD
			[], [], 'b-',			# DH
			[], [], 'b-',			# FG
			[], [], 'b-',			# HG
			[], [], 'b-',)			# IH
	
	moving, = ax.plot([], [], 'ro')
	
	def init_point():
		moving.set_data([], [])
		return moving,
	
	def update(frame):
		
		moving.set_data([x8[frame]], [y8[frame]])
		
		lines[0].set_data([xA[frame], xB[frame]], [yA[frame], yB[frame]])  
		lines[1].set_data([xB[frame], xC[frame]], [yB[frame], yC[frame]])
		lines[2].set_data([xA[frame], xE[frame]], [yA[frame], yE[frame]])
		lines[3].set_data([xA[frame], xF[frame]], [yA[frame], yF[frame]])
		lines[4].set_data([xE[frame], xF[frame]], [yE[frame], yF[frame]])
		lines[5].set_data([xC[frame], xD[frame]], [yC[frame], yD[frame]])
		lines[6].set_data([xD[frame], xH[frame]], [yD[frame], yH[frame]])
		lines[7].set_data([xF[frame], xG[frame]], [yF[frame], yG[frame]])
		lines[8].set_data([xH[frame], xG[frame]], [yH[frame], yG[frame]])  
		lines[9].set_data([xH[frame], x8[frame]], [yH[frame], y8[frame]])  

		return [moving, *lines]
	
	ani1 = animation.FuncAnimation(fig, update, frames=len(points_list), init_func=init_point, blit=True, interval=2)
	
	plt.show()


    

	








