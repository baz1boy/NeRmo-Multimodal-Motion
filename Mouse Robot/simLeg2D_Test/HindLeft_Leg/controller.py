import numpy as np
from q_angle_calculation import servo_value
from joint_position import compute_foot_end_position



class LegController(object):
	getControl = servo_value()
	
	def __init__(self, starter, cycle, time_step):
		super(LegController, self).__init__()
		self.start = starter
		self.cycle = cycle
		self.time_step = time_step

		self.q1_val, self.q2_val, self.stepNum = self.getControl.q_values(starter, cycle, time_step)
		self.Iy_list, self.Iz_list = compute_foot_end_position(self.q1_val, self.q2_val)


	def runCtrl(self, cur_step, leg_ID):

		self.cur_step = cur_step % self.stepNum
		self.step_num = self.cur_step
		q1 = self.q1_val[self.step_num]
		q2 = self.q2_val[self.step_num]
		q_val = [q1, q2]

		return q_val
	
	def pos_foot_end(self):
		I_pos_y = self.Iy_list[self.step_num]
		I_pos_z = self.Iz_list[self.step_num]
		I_pos = [I_pos_y, I_pos_z]
		return I_pos

	

