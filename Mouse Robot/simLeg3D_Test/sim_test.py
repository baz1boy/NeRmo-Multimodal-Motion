from simControl import simModel
from controller import LegController
import leg3d_Gait as lg
import numpy as np

INIT_STEPS = 1000
RUN_TIME_LENGTH = 20
SIM_STEPS = int(RUN_TIME_LENGTH/0.002)
if __name__ == '__main__':

	theLeg = simModel("models/leg_prototype_stl.xml")
	lc = LegController()
	T = 2
	timeStep = 0.002

	q_start = np.array([np.radians(20), np.radians(90), np.radians(0)])
	init_pos = lc.init_pos
	init_q = lc.init_q

	for i in range(INIT_STEPS):
		ctrlData = q_start - init_q
		theLeg.ctrlServo(ctrlData, timeStep)

	p_t = init_pos

	# locomotion, index = 1
	p_f = np.array([0.045, -0.015, -0.050])
	for i in range(500):
		getData, p_loco = lc.straightCtrl(p_t, p_f, i)
		ctrlData = getData - init_q

		theLeg.ctrlServo(ctrlData, timeStep)

	p_t = p_loco
	for i in range(100):
		q1, q2, q3, p_reset = lc.resetCtrl(p_t)
		ctrlData = np.array([q1[i], q2[i], q3[i]]) - init_q

		theLeg.ctrlServo(ctrlData, timeStep)

	p_t = np.array(p_reset[99,:])
	for i in range(400):
		delta_x = 0.01
		getData, p_side = lc.sideCtrl(delta_x, p_t, i)
		ctrlData = getData - init_q

		theLeg.ctrlServo(ctrlData, timeStep)	

		

