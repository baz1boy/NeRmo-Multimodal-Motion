import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def cubic_bezier(y0, yf, x):
    assert 0 <= x <= 1, "t must be between 0 and 1"
    y_diff = yf - y0
    bezier = x**3 + 3 * x**2 * (1 - x)
    return y0 + bezier * y_diff

def compute_swing_trajectory_bezier(phase, p0, pf, height):
    p = cubic_bezier(p0[:2], pf[:2], phase)  # 仅对x和y坐标应用贝塞尔曲线
    if phase < 0.5:
        zp = cubic_bezier(p0[2], p0[2] + height, phase * 2)
    else:
        zp = cubic_bezier(p0[2] + height, pf[2], phase * 2 - 1)
    return np.array([*p, zp])

# 初始化起始点和终止点
pf = np.array([45, -15, -50])  # 起始位置
p0 = np.array([32.5, -39, -50])  # 结束位置
height = 0  # 足部摆动的最大高度

# 生成轨迹点
points = np.array([compute_swing_trajectory_bezier(phase, p0, pf, height) for phase in np.linspace(0, 1, 100)])

# 绘制轨迹
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(points[:,0], points[:,1], points[:,2], label='Stance Trajectory')
ax.scatter(p0[0], p0[1], p0[2], color='red', s=50, label='Start Point')  # 标记起始点为红色
ax.scatter(pf[0], pf[1], pf[2], color='blue', s=50, label='End Point')  # 标记结束点为蓝色

# 设置坐标轴范围
max_range = np.array([points[:,0].max()-points[:,0].min(), points[:,1].max()-points[:,1].min(), points[:,2].max()-points[:,2].min()]).max() / 2.0
mid_x = (points[:,0].max()+points[:,0].min()) * 0.5
mid_y = (points[:,1].max()+points[:,1].min()) * 0.5
mid_z = (points[:,2].max()+points[:,2].min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# 保持坐标轴比例一致
ax.set_box_aspect([1,1,1])  # 只在一些matplotlib版本中可用

ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.legend()
plt.savefig('vector_plot.pdf')
plt.show()
