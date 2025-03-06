import socket
import numpy as np
import time

'''
ID:
0  --> Head Horizontal		1  --> Head Vertical

2  --> Fore-Left Leg		3  --> Fore-Left Coil		4  --> Fore-Left Shoulder
5  --> Fore-Right Leg		6  --> Fore-Right Coil		7  --> Fore-Right Shoulder
8  --> Hind-Left Leg		9  --> Hind-Left Coil		10 --> Hind-Left Hip
11 --> Hind-Right Leg		12 --> Hind-Right Coil		13 --> Hind-Right Hip
Unit: degree [0, 180]

14 --> Spine DOF
15 --> Tail DOF
'''

class RobotDriver(object):
	"""docstring for RobotDriver"""
	MotorNum = 14   # without spine and tail control
	TimeStep = 0.02 # Unit -> Second
	
	def __init__(self, robot_ip, robot_port, timeStep):
		super(RobotDriver, self).__init__()
		
		## For Connection
		self.TimeStep = timeStep
		self.s_address = (robot_ip, robot_port)
		self.theRobot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		
		## For Control
		"""
		Transfer:
					0  Head h  : * +1 		
					1  Head v  : * +1 

		2  LF Leg  : * +1 		5  RF Leg  : * +1
		3  LF Coil : * +1 		6  RF Coil : * +1
		4  LF Shou : * +1		7  RF Shou : * +1

		8  LH Leg  : * +1 		11 RH Leg  : * +1
		9  LH Coil : * +1 		12 RH Coil : * +1
		10 LH Hip  : * +1       13 RH Hip  : * +1

		"""
		self.sim2real = {
			"leg":[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
		}
		self.angleList = [0]*self.MotorNum
		self.ctrlData = [0]*self.MotorNum
		#self.initAngle = [0,0, 0,90,0, 0,90,0, 0,90,0, 0,90,0]

	def runStep(self, q_data, start_time):
		for i in range(self.MotorNum):
			self.angleList[i] = q_data[i]
		
		##
		for i in range(self.MotorNum):
			self.ctrlData[i] = self.angleList[i]

		#print(self.ctrlData)
		returnData = self.toInteract(self.ctrlData)

		timeCost = time.time()-start_time
		if timeCost < self.TimeStep:
			time.sleep(self.TimeStep-timeCost)

	def toInteract(self, sendData):
		stringData = ""
		for i in range(self.MotorNum):
			tStr = str(int(sendData[i]))
			stringData = stringData + tStr + ","
		self.theRobot.sendto(stringData.encode(), self.s_address)
		data_r = []
		for i in range(10):
			try:
				self.theRobot.settimeout(0.02)
				data_r, addr = self.theRobot.recvfrom(1024)  # Receive feedback from robot
				data_r = data_r.decode()
				break
			except socket.timeout:
				print("UDP Time out --> ", i)
				self.theRobot.sendto(stringData.encode(), self.s_address)
		return data_r

	def shutdown(self):
		self.theRobot.close()