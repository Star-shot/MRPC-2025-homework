# Quadrotor Differential Flatness Package

## 功能说明

本ROS包实现了四旋翼无人机的微分平坦性计算，根据给定的双纽线轨迹计算无人机在整个飞行过程中的姿态四元数。

## 编译方法

```bash
cd ~/MRPC-2025-homework/code
catkin_make
source devel/setup.bash
```

## 使用方法

### 基本用法

```bash
rosrun quadrotor_df compute_quaternion [输出文件路径]
```

如果不指定输出路径，默认输出到 `$HOME/MRPC-2025-homework/solutions/df_quaternion.csv`

### 示例

```bash
# 使用默认路径
rosrun quadrotor_df compute_quaternion

# 指定输出路径
rosrun quadrotor_df compute_quaternion /path/to/output/df_quaternion.csv
```

## 输出格式

生成的CSV文件格式如下：

```
t, x, y, z, w
0.00, 0.0499792, 0.0000000, 0.0000000, 0.9987503
0.02, 0.0499167, 0.0024979, 0.0000000, 0.9975021
...
```

- 时间 `t`：保留2位小数
- 四元数 `x, y, z, w`：保留7位小数
- 四元数已归一化且满足 $w \geq 0$

## 轨迹参数

- 时间范围：$t \in [0, 2\pi)$
- 采样步长：$\Delta t = 0.02$ 秒
- 重力加速度：$g = 9.81$ m/s²

## 理论背景

根据微分平坦性理论，四旋翼的姿态可以通过平坦输出（位置和偏航角）及其导数计算得到：

1. **前向轴（x轴）**：与速度方向对齐
2. **上向轴（z轴）**：指向期望加速度方向（轨迹加速度 + 重力）
3. **左侧轴（y轴）**：根据右手坐标系计算（$e_y = e_z \times e_x$）

