import numpy as np
import leg3d_Gait as gait


class LegController(object):

	
	def __init__(self):
		super(LegController, self).__init__()
		self.init_pos = np.array([0.0325, -0.0396388, -0.050])
		self.init_q = np.array([np.radians(20), np.radians(90), np.radians(0)])


	def resetCtrl(self, p):
		p0 = np.array(p)
		pf = self.init_pos 

		re_q1, re_q2, re_q3, re_I = gait.resetControl(p0, pf)
		
		return np.radians(re_q1), np.radians(re_q2), np.radians(re_q3), re_I
	
	def straightCtrl(self, p1, p2, step):
		n = step % 200 #step number
		if n < 100:
			q1, q2, q3, p_out = gait.locomotion(p1, p2, 1)
		elif 100 <= n < 200:
			q1, q2, q3, p_out = gait.locomotion(p1, p2, 0)
			n = n - 100
	
		getData = np.array([np.radians(q1[n]), np.radians(q2[n]), np.radians(q3[n])])
			
		return getData, np.array(p_out[n])
	
	def sideCtrl(self, delta_x, p, step):
		n = step % 200 #step number
		if n < 100:
			q1, q2, q3, p_out = gait.sideShift(delta_x, p, 1)
		elif 100 <= n < 200:
			q1, q2, q3, p_out = gait.sideShift(delta_x, p, 0)
			n = n - 100

		getData = np.array([np.radians(q1[n]), np.radians(q2[n]), np.radians(q3[n])])
			
		return getData, np.array(p_out[n])
		

		
	

	

