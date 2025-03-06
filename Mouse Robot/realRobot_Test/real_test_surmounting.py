from real_control import RobotDriver
from real_controller import robotController, q2p, p2q
import time
import numpy as np

# INIT_STEPS = 
if __name__ == '__main__':
	timeStep = 0.002  # Unit: s

	mouse = RobotDriver('192.168.2.84', 6666, timeStep)
	print("Robot Connected !!!")

	T = 2  # frequency = 0.5 
	RC = robotController()

	init_diff = RC.init_qdiff  	# due to model setup, modeling data
	pleg = RC.init_pleg
	pbody = RC.init_pbody
	angle = RC.init_angle
	center = RC.init_center

	print("Controller initiated !!!")

	## Mouse Initialization
	q0 = np.array([0,0])
	q_init = RC.init_q
	q_init = np.hstack((q0,q_init))
	init_diff = np.hstack((q0,init_diff))
	
	print("Step 1 -->")
	t_init = 100
	for i in range(t_init):
		start_time = time.time()
		ctrlData = q_init
		mouse.runStep(ctrlData, start_time)

	print("Step 2 -->")
	q_initialization = np.array([np.linspace(q_init[i], q_init[i], int(t_init/2)) for i in range(14)]).T
	for i in range(t_init):
		if i<(t_init/2):
			start_time = time.time()
			ctrlData = q_initialization[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_initialization[-1]
			mouse.runStep(ctrlData, start_time)
		
	print("Robot initialized !!!")
	
	 ## Surmounting an Obstacle Motion
	p0 = np.array([[0.0325, 0.0039325, -0.08], [-0.0325, 0.0039325, -0.08], 
				 	[0.0325, 0.0039325, -0.08], [-0.0325, 0.0039325, -0.08]])
	p1 = np.array([[0.0325, -0.0160675, -0.08], [-0.0325, -0.0160675, -0.08], 
					[0.0325, -0.0160675, -0.08], [-0.0325, -0.0160675, -0.08]])		
	pm = (p0+p1)/2
	qm = p2q(pm)
	q_head = np.array([0,0])
	q_start = np.hstack((q_head, qm))

    ## Start up
	t_startup = 200
	q_startup = np.array([np.linspace(q_initialization[-1][i], q_start[i], int(t_startup/2)) for i in range(14)]).T
	for i in range(t_startup):
		if i<(t_startup/2):
			start_time = time.time()
			ctrlData = q_startup[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_startup[-1]
			mouse.runStep(ctrlData, start_time)
	pleg_cur = q2p(q_startup[-1])

	h_lift = 0.005
	q_move = RC.moveControl(p0, p1, h_lift)
	n = len(q_move)

	t_transtion = 200
	q_transiton = np.array([np.linspace(q_startup[-1][i], q_move[0][i], int(t_transtion/2)) for i in range(14)]).T
	for i in range(t_transtion):
		if i<(t_transtion/2):
			start_time = time.time()
			ctrlData = q_transiton[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_transiton[-1]
			mouse.runStep(ctrlData, start_time)

	t_move = 500
	q_reach = np.zeros_like(q_transiton[-1])
	for i in range(t_move):
		stepNUM = i % n
		start_time = time.time()
		ctrlData = q_move[stepNUM]
		mouse.runStep(ctrlData, start_time)
	
	t_pause = 100
	for i in range(t_pause):
		start_time = time.time()
		ctrlData = q_move[stepNUM]
		mouse.runStep(ctrlData, start_time)

	# move1:
	pleg_cur = q2p(q_reach)
	q_move1 = RC.surmountControl(pleg_cur, 1)
	t_move1 = 400
	for i in range(t_move1):
		stepNUM = i % len(q_move1)
		start_time = time.time()
		ctrlData = q_move1[stepNUM]
		mouse.runStep(ctrlData, start_time)
	pleg_cur2 = q2p(q_move1[stepNUM])

	# move2:
	q_move2 = RC.surmountControl(pleg_cur2,2)
	t_move2 = 800
	for i in range(t_move2):
		stepNUM = i % len(q_move2)
		start_time = time.time()
		ctrlData = q_move2[stepNUM]
		q_continue = q_move2[stepNUM]
		mouse.runStep(ctrlData, start_time)
	
	t_pause = 50
	for i in range(t_pause):
		start_time = time.time()
		ctrlData = q_continue
		mouse.runStep(ctrlData, start_time)
	pleg_cur3 = q2p(q_continue)

	# move3: 
	q_move3 = RC.surmountControl(pleg_cur3,3)
	t_adj = 200
	q_adj = np.array([np.linspace(q_continue[i], q_move3[0][i], int(t_adj)) for i in range(14)]).T
	for i in range(t_adj+100):
		if i<(t_adj):
			start_time = time.time()
			ctrlData = q_adj[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_adj[-1]
			mouse.runStep(ctrlData, start_time)

	t_move3 = 900
	#print(333)
	for i in range(t_move3):
		stepNUM = i % len(q_move3)
		start_time = time.time()
		ctrlData = q_move3[stepNUM]
		mouse.runStep(ctrlData, start_time)
	pleg_cur4 = q2p(q_move3[stepNUM])

	# move 4:
	q_move4 = RC.surmountControl(pleg_cur4,4)
	t_move4 = 400
	#print(444)
	for i in range(t_move4):
		stepNUM = i % len(q_move4)
		start_time = time.time()
		ctrlData = q_move4[stepNUM]
		mouse.runStep(ctrlData, start_time)

	t_pause = 10
	for i in range(t_pause):
		start_time = time.time()
		ctrlData = q_move4[stepNUM]
		mouse.runStep(ctrlData, start_time)
	pleg_cur5 = q2p(q_move4[stepNUM])

	q_move6 = RC.surmountControl(pleg_cur5,6)
	t_move6 = 300
	#print(666)
	for i in range(t_move6):
		stepNUM = i % len(q_move6)
		start_time = time.time()
		ctrlData = q_move6[stepNUM]
		q_continue = q_move6[stepNUM]
		mouse.runStep(ctrlData, start_time)

	t_pause = 50
	for i in range(t_pause):
		start_time = time.time()
		ctrlData = q_continue
		mouse.runStep(ctrlData, start_time)
	pleg_cur7 = q2p(q_continue)

	# move 7:
	q_move7 = RC.surmountControl(pleg_cur7,7)
	t_adj = 200
	q_adj = np.array([np.linspace(q_continue[i], q_move7[0][i], t_adj) for i in range(14)]).T
	for i in range(t_adj+100):
		if i<(t_adj):
			start_time = time.time()
			ctrlData = q_adj[i]
			mouse.runStep(ctrlData, start_time)
		else:
			start_time = time.time()
			ctrlData = q_adj[-1]
			mouse.runStep(ctrlData, start_time)

	t_move7 = 2400			##reach ground FL
	for i in range(t_move7):
		stepNUM = i % len(q_move7)
		start_time = time.time()
		ctrlData = q_move7[stepNUM]
		mouse.runStep(ctrlData, start_time)

	# # move 8:			##reach ground HL
	# pleg_cur8 = q2p(q_move7[stepNUM])
	# q_move8 = RC.surmountControl(pleg_cur8,8)
	# t_move8 = 500
	# for i in range(t_move8):
	# 	stepNUM = i % len(q_move8)
	# 	start_time = time.time()
	# 	ctrlData = q_move8[stepNUM]
	# 	mouse.runStep(ctrlData, start_time)

	# pleg_cur9 = q2p(q_move8[stepNUM])
	# q_move9 = RC.surmountControl(pleg_cur9,9)
	# t_move9 = 1000
	# for i in range(t_move9):
	# 	stepNUM = i % len(q_move9)
	# 	start_time = time.time()
	# 	ctrlData = q_move9[stepNUM]
	# 	mouse.runStep(ctrlData, start_time)
	
	
	mouse.shutdown()
		

