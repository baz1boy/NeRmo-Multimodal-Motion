from matplotlib import animation
import numpy as np
from matplotlib import pyplot as plt
from leg3d_Joint import computer_joint_3d
from leg3d_Gait import heightControl, sideShift, locomotion


t = np.linspace(0, 1, 100)   # t[0, 1]

###swing tragectory test
p0 = np.array([0.045, -0.015, -0.050])  
pf = np.array([0.0325, -0.039, -0.050])  
height = 0.010

q1, q2, q3, I = locomotion(p0, pf, 0)

###height control test
# delta_h = 0.02
# p_cur = [0.045, -0.015, -0.05]
# q1, q2, q3, I = heightControl(delta_h, p_cur)

###side shift test
# delta_x = -0.01
# p_cur = [0.045, -0.015, -0.05]
# q1, q2, q3, I = sideShift(delta_x, p_cur, 0)



# here calculate the joint position, and plot animation
O = np.zeros((len(t), 3))
A = np.zeros((len(t), 3))
B = np.zeros((len(t), 3))
C = np.zeros((len(t), 3))
D = np.zeros((len(t), 3))
E = np.zeros((len(t), 3))
F = np.zeros((len(t), 3))
G = np.zeros((len(t), 3))
H = np.zeros((len(t), 3))
I = np.array(I)

for i in range(len(t)):
    A[i,:], B[i,:], C[i,:], D[i,:], E[i,:], F[i,:], G[i,:], H[i,:] = np.array(computer_joint_3d(q1[i], q2[i], q3[i]))


#print(len(I_swing))


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(I[:,0], I[:,1], I[:,2], lw=2, label='Swing Trajectory')
ax.set_xlim(-0.05, 0.1)  
ax.set_ylim(-0.1, 0.05)
ax.set_zlim(-0.1, 0.05)
ax.set_xlabel("x data")
ax.set_ylabel("y data")
ax.set_zlabel("z data")
ax.grid(True)	

lines = plt.plot([], [], 'r-',                  # 0A
                 [], [], 'k-',                  # AB
                 [], [], 'r-',			# BC
                 [], [], 'r-',			# AE
                 [], [], 'r-',			# AF
                 [], [], 'r-',			# EF
                 [], [], 'b-',			# CD
                 [], [], 'b-',			# DH
                 [], [], 'b-',			# FG
                 [], [], 'b-',			# HG
                 [], [], 'b-',)			# IH

moving, = ax.plot([], [], [], 'ro')

def init_point():
        moving.set_data_3d([], [], [])
        return moving,

def update(frame):
        
        moving.set_data_3d([I[frame,0]], [I[frame,1]], [I[frame,2]])

        lines[0].set_data_3d([O[frame,0], A[frame,0]], [O[frame,1], A[frame,1]], [O[frame,2], A[frame,2]])
        lines[1].set_data_3d([A[frame,0], B[frame,0]], [A[frame,1], B[frame,1]], [A[frame,2], B[frame,2]])  
        lines[2].set_data_3d([B[frame,0], C[frame,0]], [B[frame,1], C[frame,1]], [B[frame,2], C[frame,2]])
        lines[3].set_data_3d([A[frame,0], E[frame,0]], [A[frame,1], E[frame,1]], [A[frame,2], E[frame,2]])
        lines[4].set_data_3d([A[frame,0], F[frame,0]], [A[frame,1], F[frame,1]], [A[frame,2], F[frame,2]])
        lines[5].set_data_3d([E[frame,0], F[frame,0]], [E[frame,1], F[frame,1]], [E[frame,2], F[frame,2]])
        lines[6].set_data_3d([C[frame,0], D[frame,0]], [C[frame,1], D[frame,1]], [C[frame,2], D[frame,2]])
        lines[7].set_data_3d([D[frame,0], H[frame,0]], [D[frame,1], H[frame,1]], [D[frame,2], H[frame,2]])
        lines[8].set_data_3d([F[frame,0], G[frame,0]], [F[frame,1], G[frame,1]], [F[frame,2], G[frame,2]])
        lines[9].set_data_3d([H[frame,0], G[frame,0]], [H[frame,1], G[frame,1]], [H[frame,2], G[frame,2]])  
        lines[10].set_data_3d([I[frame,0], H[frame,0]], [I[frame,1], H[frame,1]], [I[frame,2], H[frame,2]])  

        return [moving, *lines]

ani1 = animation.FuncAnimation(fig, update, frames=len(t), init_func=init_point, blit=True, interval=10)

plt.show()
    