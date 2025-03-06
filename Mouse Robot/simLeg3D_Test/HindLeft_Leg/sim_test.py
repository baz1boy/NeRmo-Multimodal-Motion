from Sim_Control import SimModel
from controller import LegController
import matplotlib.pyplot as plt
import numpy as np
import time

INIT_STEPS = 1000
RUN_TIME_LENGTH = 20
SIM_STEPS = RUN_TIME_LENGTH/0.002
if __name__ == '__main__':

	theLeg = SimModel("leg_prototype_no.xml")

	start_pos = [-0.044, -0.060]
	T = 2
	time_step = 0.002
	theController = LegController(start_pos, T, time_step)

	step_per_cycle = theController.stepNum
	q1_val = theController.q1_val
	q2_val = theController.q2_val

	init_joint_pos_1 = q1_val[0]
	init_joint_pos_2 = q2_val[0]
	diff1 = init_joint_pos_1 - 20
	diff2 = init_joint_pos_2 - 90
	diff_jointA = np.radians(diff1)
	diff_jointB = np.radians(diff2)

	# print(step_per_cycle)
	# print(len(q2_val))
	
	for i in range(INIT_STEPS):
		ctrlData = [diff_jointA, diff_jointB]
		theLeg.ctrl_servo(ctrlData, time_step) 
		

	for i in range(SIM_STEPS):
		cur_step = i % step_per_cycle
		print(cur_step)
		cur_q_A = q1_val[cur_step]
		cur_q_B = q2_val[cur_step]
		ctrlA = cur_q_A - 20
		ctrlB = cur_q_B - 90
		ctrl_jointA = np.radians(ctrlA)
		ctrl_jointB = np.radians(ctrlB)
		ctrlData_m = [ctrl_jointA, ctrl_jointB]
		theLeg.ctrl_servo(ctrlData_m, time_step)

	#'''
	# plt.plot(theController.trgList_y, theController.trgList_z)
	# plt.plot(theLeg.legRealPoint_y, theLeg.legRealPoint_z)
	
	# plt.show()