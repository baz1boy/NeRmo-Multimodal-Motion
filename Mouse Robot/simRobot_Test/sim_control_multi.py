from mujoco_py import load_model_from_path, MjSim, MjViewer
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

class simModel(object):
	
	def __init__(self, modelPath):
		super(simModel, self).__init__()
		self.model = load_model_from_path(modelPath)
		self.sim = MjSim(self.model)
		self.viewer = MjViewer(self.sim)

		self.viewer.cam.lookat[0] = 0.5
		self.viewer.cam.lookat[1] = 0
		self.viewer.cam.lookat[2] = 0.5
		self.viewer.cam.elevation = -45
		self.viewer.cam.azimuth = 180
		self.viewer.cam.distance = self.model.stat.extent*0.05

		self.sim_state = self.sim.get_state()
		self.sim.set_state(self.sim_state)

		### Check XML Body List::   # body 'mouse' index is [1]
		# for i in range(len(self.sim.model.body_names)):
		# 	print(f"Body index: {i}, Body name: {self.sim.model.body_names[i]}, Position: {self.sim.data.xipos[i]}")

		self.comx = []
		self.comy = []

		self.torso_center_x = []
		self.torso_center_y = []
		self.torso_shoulder_x = []
		self.torso_shoulder_y = []
		self.torso_hip_x = []
		self.torso_hip_y = []

		self.footend_fl_x = []
		self.footend_fr_x = []
		self.footend_hl_x = []
		self.footend_hr_x = []
		self.footend_fl_y = []
		self.footend_fr_y = []
		self.footend_hl_y = []
		self.footend_hr_y = []

		self.gyro = []
		self.acc = []


	def getSensor(self):
		com = self.sim.data.xipos
		sensordata = self.sim.data.sensordata
		#print(sensordata)
		acc = sensordata[-6:-3]
		gyro = sensordata[-3:]
		return acc, gyro
	
	def ctrlViewer(self):
		center_pos = self.sim.data.get_site_xpos('imu')
		self.viewer.cam.lookat[:] = center_pos
		self.viewer.cam.elevation = -45
		self.viewer.cam.azimuth = 180
		self.viewer.cam.distance = 1
		# self.viewer.cam.lookat[0] = self.viewer.cam.lookat[0]
		# self.viewer.cam.lookat[1] = self.viewer.cam.lookat[1]-0.00024
		# self.viewer.cam.lookat[2] = self.viewer.cam.lookat[2]
		return 
	
	def squeezingPosition(self):
		pos_door = self.sim.data.get_site_xpos('squeezing')
		pos_robot = self.sim.data.get_site_xpos('nose')
		d_start = pos_robot[0] - pos_door[0]
		return d_start
	
	def holePosition(self):
		pos_hole = self.sim.data.get_site_xpos('hole')
		pos_tail = self.sim.data.get_site_xpos('robot_tail')
		pass_hole = pos_tail[0] - pos_hole[0]
		return pass_hole
	
	def spinePosition(self):
		pos_spine = self.sim.data.get_site_xpos('robot_top_spine')
		return pos_spine[2]

	def spineholePos(self):
		pos_spine = self.sim.data.get_site_xpos('robot_top_spine')
		pos_hole = self.sim.data.get_site_xpos('hole')
		half_passed = pos_spine[0] - pos_hole[0]
		return half_passed

	def crawlingSpeed(self):
		pos_door = self.sim.data.get_site_xpos('squeezing')
		top1 = self.sim.data.get_site_xpos('robot_top_left')
		top2 = self.sim.data.get_site_xpos('robot_top_right')
		if top1[1]<=top2[1]:
			top_y = top1[1]
		else:
			top_y = top2[1]
		
		reach_door = top_y-0.001 - pos_door[1]
		return reach_door
	
	def topPosition(self):
		top1 = self.sim.data.get_site_xpos('robot_top_left')
		top2 = self.sim.data.get_site_xpos('robot_top_right')
		bottom = self.sim.data.get_site_xpos('body_bottom')
		if top1[2]>=top2[2]:
			top_z = top1[2]
		else:
			top_z = top2[2]
		return top_z
	
	def facePosition(self):
		pos_face = self.sim.data.get_site_xpos('nose')
		pos_hole = self.sim.data.get_site_xpos('squeezing')
		facing = pos_face[1] - pos_hole[1]
		return facing

	def getQuat(self):
		rot_matrix = self.sim.data.get_site_xmat('imu').reshape(3,3)
		quat = R.from_matrix(rot_matrix).as_quat()
		
		return quat
	

	def ctrlServo(self, ctrlData, timeStep):
		self.sim.data.ctrl[:] = ctrlData
		step_num = int(timeStep/0.002)
		for i in range(step_num):
			self.sim.step()
			self.viewer.render()

		com = self.sim.data.xipos[1]
		self.comx.append(-com[0])
		self.comy.append(-com[1])
	
		center = self.sim.data.get_site_xpos('imu')
		shoulder = self.sim.data.get_site_xpos('torso_shoulder')
		hip = self.sim.data.get_site_xpos('torso_hip')

		self.torso_center_x.append(-center[0])
		self.torso_shoulder_x.append(-shoulder[0])
		self.torso_hip_x.append(-hip[0])
		self.torso_center_y.append(-center[1])
		self.torso_shoulder_y.append(-shoulder[1])
		self.torso_hip_y.append(-hip[1])

		footend_fl = self.sim.data.get_site_xpos('footend_fl')
		footend_fr = self.sim.data.get_site_xpos('footend_fr')
		footend_hl = self.sim.data.get_site_xpos('footend_hl')
		footend_hr = self.sim.data.get_site_xpos('footend_hr')

		self.footend_fl_x.append(-footend_fl[0])
		self.footend_fr_x.append(-footend_fr[0])
		self.footend_hl_x.append(-footend_hl[0])
		self.footend_hr_x.append(-footend_hr[0])
		self.footend_fl_y.append(-footend_fl[1])
		self.footend_fr_y.append(-footend_fr[1])
		self.footend_hl_y.append(-footend_hl[1])
		self.footend_hr_y.append(-footend_hr[1])

		sensordata = self.sim.data.sensordata
		acc = sensordata[-6:-3]
		gyro = sensordata[-3:]
		self.acc.append(acc)
		self.gyro.append(gyro)

		
	def getTime(self):
		return self.sim.data.time
