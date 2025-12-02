import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# 参数
omega = 0.5  # rad/s
alpha = np.pi / 12

# 读取数据
df = pd.read_csv('tracking.csv')
t = df['t'].values
quat_uav = df[['qx', 'qy', 'qz', 'qw']].values  # [x, y, z, w]

# 计算执行器相对机体的旋转矩阵
def R_BD(t):
    c, s = np.cos(omega * t), np.sin(omega * t)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca, s * sa],
        [s, c * ca, -c * sa],
        [0, sa, ca]
    ])

# 计算世界坐标系下执行器姿态
quat_result = []
for i, ti in enumerate(t):
    # 无人机姿态 (scipy用xyzw格式)
    R_WB = R.from_quat(quat_uav[i]).as_matrix()
    # 世界系下执行器姿态
    R_WD = R_WB @ R_BD(ti)
    q = R.from_matrix(R_WD).as_quat()  # [x, y, z, w]
    # 确保 qw >= 0
    if q[3] < 0:
        q = -q
    quat_result.append(q)

quat_result = np.array(quat_result)

# 绘图
fig, ax = plt.subplots(figsize=(10, 6))
labels = ['$q_x$', '$q_y$', '$q_z$', '$q_w$']
for i in range(4):
    ax.plot(t, quat_result[:, i], label=labels[i], linewidth=1.5)

ax.set_xlabel('Time (s)', fontsize=12)
ax.set_ylabel('Quaternion', fontsize=12)
ax.set_title('End-Effector Orientation in World Frame', fontsize=14)
ax.legend(fontsize=11)
ax.grid(True, alpha=0.3)
ax.tick_params(labelsize=11)
plt.tight_layout()
plt.savefig('S1_quaternion.png', dpi=150)
plt.show()


