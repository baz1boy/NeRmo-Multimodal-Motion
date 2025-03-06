import numpy as np
from scipy.spatial.transform import Rotation as R
import leg_Gait as gait
from leg_Joint import computefootend_l, computefootend_r
from body_kinematics import bodyIK
import body_posture as pos


class robotController(object):
	
	def __init__(self):
		super(robotController, self).__init__()
		self.init_pleg_ = np.array([[0.0325, -0.0396388, -0.050, 1], [-0.0325, -0.0396388, -0.050, 1],
								[0.0325, -0.0396388, -0.050, 1],[-0.0325, -0.0396388, -0.050, 1]])			#this is the model setup position/angle
		self.init_pleg = np.array([[0.0325, -0.0060675, -0.055, 1], [-0.0325, -0.0060675, -0.055, 1],
								[0.0325, -0.0060675, -0.056, 1],[-0.0325, -0.0060675, -0.056, 1]])  #0.006075
		self.init_qdiff = np.array([0,90,0,0,90,0,0,90,0,0,90,0])
		self.init_angle = np.array([0,0,0])
		self.init_center = np.array([0,0,0])
		self.init_pbody = np.array([leg2body(0, self.init_pleg[0], self.init_angle, self.init_center),
							  		leg2body(1, self.init_pleg[1], self.init_angle, self.init_center),
							  		leg2body(2, self.init_pleg[2], self.init_angle, self.init_center),
							  		leg2body(3, self.init_pleg[3], self.init_angle, self.init_center)])

		self.init_q = p2q(self.init_pleg)

		PI = np.pi
		self.stepNum = 100			# default period T=2s, 0.002x100
		self.cycle = 1
		# [LF, RF, LH, RH]
		# --------------------------------------------------------------------- #
		'''
		self.phaseDiff = [0, PI, PI*1/2, PI*3/2]	# Walk
		#self.phaseDiff = [0, PI, PI*3/2, PI*1/2]	# Walk
		self.period = 3/2
		self.SteNum = 64							#32 # Devide 2*PI to multiple steps
		self.spinePhase = self.phaseDiff[3]
		'''
		# --------------------------------------------------------------------- #

		self.phaseDiff = [0, PI, PI, 0]			# Trot
		self.phaseDiff_lateral = [0, PI, 0, PI]			# Trot
		#self.phaseDiff = [PI, 0, PI,0]			# Trot
		#self.phaseDiff = [0, PI, PI*1/2, PI*3/2]	# Walk
		#self.phaseDiff = [0, PI, PI*3/2, PI*1/2]	# Walk
		self.period = 2/2
		self.fre = 1/self.cycle

		self.stepDiff = [0, 0, 0, 0]
		for i in range(4):
			self.stepDiff[i] = int(self.stepNum * self.phaseDiff[i] / (2*PI))
									
	
	def turnRotation(self, deg, pleg, angle, center):
		"""
		param: 
		deg: turning degree
		pleg: footend in leg coordinate, before move
		angle: current body pitch roll yaw

		"""
		(omega, phi, psi) = angle
		if deg>=0:
			direction = 0
			step = 15
			n, rest = np.divmod(deg, step)
		else:
			direction = 1
			step = -15
			n, rest = np.divmod(deg, step)

		angle_step = np.array([omega, phi, psi+step])
		angle_rest = np.array([omega, phi, psi+rest])

		pbody = np.zeros((4,4))  	# footend in body coordinate
		for i in range(4):
			pbody[i] = leg2body(i, pleg[i], angle, center)

		q_rest = pos.turning(direction, pleg, pbody, angle_rest, center)
		q_step = pos.turning(direction, pleg, pbody, angle_step, center)
		#print(ep)q_rest)
		#print(q_st
		
		n = int(n)
		if rest == 0:
			q_data = np.vstack([q_step]*n)
		elif rest!=0 and n==0:
			q_data = q_rest
		elif rest!=0 and n!=0:
			q_Step = np.vstack([q_step]*n)
			q_Rest = q_rest
			q_data = np.vstack((q_Step, q_Rest))

		return q_data 
	
	def pryStance(self, pleg, move, angle, center):
		pbody = np.zeros((4,4))  	# footend in body coordinate
		for i in range(4):
			pbody[i] = leg2body(i, pleg[i], angle, center)

		#print(pbody)
		angle_cur = np.array(angle) + np.array(move)

		q_out, pleg_cur = pos.pitchrollyaw(pleg, pbody, angle_cur, center)
		q_head = np.zeros((len(q_out), 2))
		q_data = np.hstack((q_head, q_out))
		#print(q_data)
		return q_data, pleg_cur, angle_cur
	
	def recoverControl(self, pleg, euler, acc):
		q0 = np.array([0,0])
		q_init = np.hstack((q0,self.init_q))
		if abs(euler[1])>=50 and abs(acc[0])>=8:
			q_data = pos.fallRecovery(pleg, q_init, 0)
		elif acc[2]<=-8 and euler[1]<0:
			q_data = pos.fallRecovery(pleg, q_init, 1)
		elif acc[2]<=-8 and euler[1]>0:
			q_data = pos.fallRecovery(pleg, q_init, 2)
		else:
			q_data = np.hstack((q0,p2q(pleg)))

		return q_data
	
	def heightCtrl(self, pleg_1, h):
		pleg = pleg_1[:, :-1]
		delta_h = pleg[0][2] + h
		q_leg = pos.verticalMove(-delta_h, pleg)
		q_head = np.zeros((len(q_leg), 2))
		q_data = np.hstack((q_head, q_leg))

		return q_data
	
	def straightControl(self, pleg, d, h):
		step = d*-1
		pleg0 = pleg[:, :-1]
		plegf = np.zeros_like(pleg0)
		for i in range(4):
			plegf[i] = pleg0[i] + [0, step, 0]
		
		#print(pleg0)
		#print(plegf)

		p0_left = pleg0[0]
		pf_left = plegf[0]
		p0_right = pleg0[1]
		pf_right = plegf[1]

		q_left, q_right = pos.straightMove(p0_left, pf_left, p0_right, pf_right, h)
		q_left[:,-1] = 0
		q_right[:,-1] = 0
		#print(q_left)
		#print(q_right)

		return q_left, q_right
	
	def moveControl(self, p0, p1, h):
		q01, q02, q03 = pos.pointMove(p0[0], p1[0], h, 0)
		q11, q12, q13 = pos.pointMove(p0[1], p1[1], h, 1)
		q21, q22, q23 = pos.pointMove(p0[2], p1[2], h, 0)
		q31, q32, q33 = pos.pointMove(p0[3], p1[3], h, 1)

		q0 =  np.array([q01,q02,q03]).T
		q1 =  np.array([q11,q12,q13]).T
		q2 =  np.array([q21,q22,q23]).T
		q3 =  np.array([q31,q32,q33]).T

		q0_out = np.zeros_like(q0)
		q1_out = np.zeros_like(q1)
		q2_out = np.zeros_like(q2)
		q3_out = np.zeros_like(q3)
		n = len(q0_out)
		for i in range(n):
			q0_out[i] = q0[(i + self.stepDiff[0]) % n]
			q1_out[i] = q1[(i + self.stepDiff[1]) % n]
			q2_out[i] = q2[(i + self.stepDiff[2]) % n]
			q3_out[i] = q3[(i + self.stepDiff[3]) % n]

		q_head = np.zeros((n, 2))
		q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		return q_data
	
	def leanControl(self, pleg, angle, center, distance):

		pbody = np.zeros((4,4))  	# footend in body coordinate
		for i in range(4):
			pbody[i] = leg2body(i, pleg[i], angle, center)

		q0, q1, q2, q3 = pos.leanMove(distance, pleg, pbody, angle, center)
		q0_out = np.zeros_like(q0)
		q1_out = np.zeros_like(q1)
		q2_out = np.zeros_like(q2)
		q3_out = np.zeros_like(q3)
		n = len(q0)
		for i in range(n):
			q0_out[i] = q0[(i + self.stepDiff[0]) % n]
			q1_out[i] = q1[(i + self.stepDiff[1]) % n]
			q2_out[i] = q2[(i + self.stepDiff[2]) % n]
			q3_out[i] = q3[(i + self.stepDiff[3]) % n]

		q_head = np.zeros((n, 2))
		q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		return q_data
	
	def squeezingControl(self, index):
		"""
		index:	0			1			2
				4legs		forelegs	hindlegs
		"""
		if index == 0:
			h_hind = 0.08
			p_before_squ = np.array([[0.0325, 0.004, -h_hind], [-0.0325, 0.004, -h_hind], 
									[0.0325, 0.004, -h_hind], [-0.0325, 0.004, -h_hind]])
			
			q0, q1, q2, q3 = pos.squeezing(p_before_squ, 0)

		elif index == 1:
			p_squ = np.array([[0.045, 0.006, -0.028], [-0.045, 0.006, -0.028], 
							[0.045, 0.006, -0.027], [-0.045, 0.006, -0.027]])
			q0, q1, q2, q3 = pos.squeezing(p_squ, 1)

		elif index == 2:
			p_after_squ = np.array([[0.04, 0.01, -0.045], [-0.04, 0.01, -0.045], 
									[0.04, 0.01, -0.045], [-0.04, 0.01, -0.045]])
			q0, q1, q2, q3 = pos.squeezing(p_after_squ, 2)

		q0_out = np.zeros_like(q0)
		q1_out = np.zeros_like(q1)
		q2_out = np.zeros_like(q2)
		q3_out = np.zeros_like(q3)
		n = len(q0_out)
		for i in range(n):
			q0_out[i] = q0[(i + self.stepDiff[0]) % n]
			q1_out[i] = q1[(i + self.stepDiff[1]) % n]
			q2_out[i] = q2[(i + self.stepDiff[2]) % n]
			q3_out[i] = q3[(i + self.stepDiff[3]) % n]

		q_head = np.zeros((n, 2))
		q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))
	
		return q_data
	
	def squeezingnarrowControl(self, index):
		if index == 0:
			p_adj0 = np.array([[0.0325, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05], 
								[0.045, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05]])	
			q_out = pos.squeezingNarrow(p_adj0, 0)
			q_head = np.array([0, 0])
			q_data = np.hstack((q_head, q_out))
		elif index == 1:
			p_adj1 = np.array([[0.0325, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05], 
								[0.045, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05]])
			q_out = pos.squeezingNarrow(p_adj1, 1)
			q_head = np.zeros((len(q_out), 2))
			q_data = np.hstack((q_head, q_out))	
		elif index == 2:
			p_adj2 = np.array([[0.0325, -0.0160675, -0.0454], [-0.0325, -0.0160675, -0.0454], 
								[0.0325, -0.0160675, -0.0454], [-0.0325, -0.0160675, -0.0454]]) #-0.0374
			q_out = pos.squeezingNarrow(p_adj2, 2)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]

			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))
			q_data[:,2] = 35
			q_data[:,3] = 180
			q_data[:,4] = -90
		elif index == 3:
			p_adj3 = np.array([[0.0325, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05], 
								[0.045, -0.0160675, -0.05], [-0.0325, -0.0160675, -0.05]])
			q_out = pos.squeezingNarrow(p_adj3, 3)
			q_head = np.zeros((len(q_out), 2))
			q_data = np.hstack((q_head, q_out))	
		elif index == 4:
			p_adj4 = np.array([[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05], 
								[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05]])
			q_out = pos.squeezingNarrow(p_adj4, 4)
			q_head = np.zeros((len(q_out), 2))
			q_data = np.hstack((q_head, q_out))
		elif index == 5:
			p_adj5 = np.array([[0.0325, -0.0160675, -0.0454], [-0.0325, -0.0160675, -0.0454], 
								[0.0325, -0.0160675, -0.0454], [-0.0325, -0.0160675, -0.0454]])
			q_out = pos.squeezingNarrow(p_adj5, 5)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))
		elif index == 6:
			p_adj6 = np.array([[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05], 
								[0.0325, 0.0039325, -0.05], [-0.0325, 0.0039325, -0.05]])
			q_out = pos.squeezingNarrow(p_adj6, 6)
			q_head = np.zeros((len(q_out), 2))
			q_data = np.hstack((q_head, q_out))	

		return q_data
	
	def turnAngle(self, init_quat, cur_quat):
		# Convert quaternions to rotation matrices
		r_init = R.from_quat(init_quat)
		r_cur = R.from_quat(cur_quat)

		# Compute the relative rotation matrix
		r_relative = r_cur * r_init.inv()

		# Extract Euler angles from the relative rotation matrix
		euler_angles = r_relative.as_euler('xyz', degrees=True)

		# The rotation around the Z-axis is the third element
		z_rotation_angle = euler_angles[2]

		return z_rotation_angle
	
	def sideControl(self, n, p0, d, h):
		pf = p0.copy()
		for i in range(4):
			pf[i][0] = pf[i][0] + d
		
		q01, q02, q03 = pos.sideMove(n, p0[0], pf[0], h, 0)
		q11, q12, q13 = pos.sideMove(n, p0[1], pf[1], h, 1)
		q21, q22, q23 = pos.sideMove(n, p0[2], pf[2], h, 0)
		q31, q32, q33 = pos.sideMove(n, p0[3], pf[3], h, 1)

		q0 =  np.array([q01,q02,q03]).T
		q1 =  np.array([q11,q12,q13]).T
		q2 =  np.array([q21,q22,q23]).T
		q3 =  np.array([q31,q32,q33]).T

		q0_out = np.zeros_like(q0)
		q1_out = np.zeros_like(q1)
		q2_out = np.zeros_like(q2)
		q3_out = np.zeros_like(q3)
		n = len(q0_out)
		for i in range(n):
			q0_out[i] = q0[(i + self.stepDiff[0]) % n]
			q1_out[i] = q1[(i + self.stepDiff[1]) % n]
			q2_out[i] = q2[(i + self.stepDiff[2]) % n]
			q3_out[i] = q3[(i + self.stepDiff[3]) % n]

		q_head = np.zeros((n, 2))
		q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))
	
		return q_data
	
	def surmountControl(self, pleg_cur, index):
		h_obstacle = 0.02

		if index == 1:
			p = np.array([[0.0325, 0.02, -0.075], [-0.0325, 0.02, -0.075], 
							[0.0325, 0.02, -0.075], [-0.0325, 0.02, -0.075]])
			q_out = pos.surmountObstacle(pleg_cur, p, 1)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		elif index == 2:
			p = np.array([[0.0325, 0.02, -0.080], [-0.0325, 0.02, -0.080], 
							[0.0325, 0.02, -0.080], [-0.0325, 0.02, -0.080]])
			q_out = pos.surmountObstacle(pleg_cur, p, 2)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))
		
		elif index == 3:
			p3 = np.array([[0.0325, -0.02, -0.050], [-0.0325, -0.02, -0.050], 
							[0.0325, 0.02, -0.080], [-0.0325, 0.02, -0.080]])
			q_out = pos.surmountObstacle(pleg_cur, p3, 3)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]

			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))
		
		elif index == 4:
			p = np.array([[0.0325, -0.02, -0.040], [-0.0325, -0.02, -0.040], 
							[0.0325, 0., -0.080], [-0.0325, 0., -0.080]])		
			q_out = pos.surmountObstacle(pleg_cur, p, 4)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		elif index == 6:
			p = np.array([[0.0325, -0.02, -0.040], [-0.0325, -0.02, -0.040], 
							[0.0325, 0.02, -0.080], [-0.0325, 0.02, -0.080]])
			# for i in range(2):
			# 	p[i][2] = p[i+2][2] + h_obstacle
			q_out = pos.surmountObstacle(pleg_cur, p, 6)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		elif index == 7:
			p = np.array([[0.0325, -0.01, -0.06], [-0.0325, -0.01, -0.06], 
							[0.0325, 0.004, -0.06], [-0.0325, 0.004, -0.06]])
			q_out = pos.surmountObstacle(pleg_cur, p, 7)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		elif index == 8:
			p = np.array([[0.0375, -0.02, -0.06], [-0.0375, -0.02, -0.06], 
							[0.0325, 0.004, -0.06], [-0.0325, 0.004, -0.06]])
			for i in range(2):
				p[i+2][2] = p[i][2] + h_obstacle/2
			q_out = pos.surmountObstacle(pleg_cur, p, 8)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		elif index == 9:
			p = np.array([[0.0325, 0.004, -0.06], [-0.0325, 0.004, -0.06], 
							[0.0325, 0.004, -0.06], [-0.0325, 0.004, -0.06]])
			q_out = pos.surmountObstacle(pleg_cur, p, 7)
			q_out_split = np.split(q_out, 4, axis=1)
			q0_out = np.zeros_like(q_out_split[0])
			q1_out = np.zeros_like(q_out_split[1])
			q2_out = np.zeros_like(q_out_split[2])
			q3_out = np.zeros_like(q_out_split[3])
			n = len(q0_out)
			for i in range(n):
				q0_out[i] = q_out_split[0][(i + self.stepDiff[0]) % n]
				q1_out[i] = q_out_split[1][(i + self.stepDiff[1]) % n]
				q2_out[i] = q_out_split[2][(i + self.stepDiff[2]) % n]
				q3_out[i] = q_out_split[3][(i + self.stepDiff[3]) % n]
			q_head = np.zeros((n, 2))
			q_data = np.hstack((q_head, q0_out, q1_out, q2_out, q3_out))

		return q_data


def leg2body(legID, pleg, angle, center):
	(Tlf,Trf,Tlh,Trh,Tm) = bodyIK(angle[0], angle[1], angle[2],center[0],center[1],center[2])
	if legID==0:
		pbody = np.dot(Tlf, pleg)
	elif legID==1:
		pbody = np.dot(Trf, pleg)
	elif legID==2:
		pbody = np.dot(Tlh, pleg)
	elif legID==3:
		pbody = np.dot(Trh, pleg)

	return pbody

def p2q(pleg):
	q1_0, q2_0, q3_0, Ileg_0 = gait.standControl([pleg[0][0],pleg[0][1],pleg[0][2]],0)
	q1_1, q2_1, q3_1, Ileg_1 = gait.standControl([pleg[1][0],pleg[1][1],pleg[1][2]],1)
	q1_2, q2_2, q3_2, Ileg_2 = gait.standControl([pleg[2][0],pleg[2][1],pleg[2][2]],0)
	q1_3, q2_3, q3_3, Ileg_3 = gait.standControl([pleg[3][0],pleg[3][1],pleg[3][2]],1)
	q_cal = np.array([q1_0,q2_0,q3_0, q1_1,q2_1,q3_1, q1_2,q2_2,q3_2, q1_3,q2_3,q3_3])
	return q_cal

def q2p(q_cur):
	Ileg = np.zeros((4,3))
	Ileg[0] = computefootend_l(q_cur[2], q_cur[3], q_cur[4])
	Ileg[1] = computefootend_r(q_cur[5], q_cur[6], q_cur[7])
	Ileg[2] = computefootend_l(q_cur[8], q_cur[9], q_cur[10])
	Ileg[3] = computefootend_r(q_cur[11], q_cur[12], q_cur[13])
	ones_col = np.ones((4,1))
	pleg = np.hstack((Ileg, ones_col))
	
	return pleg
