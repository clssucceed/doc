[TOC]
# Wheel Velocity Factor
## Wheel Velocity Factor Residual推导
$$
\begin{aligned}
v^{W}_i 
&= 0.5(v_{li} + v_{ri}) \\
\hat{v}^{W}_i 
&= R_{IW}^T(R_{wI_i}^Tv^{w}_{i} + [\omega_i - bg_i]_{\times}t_{IW})\\ 
r_v
&= [v^{W}_i - \hat{v}^{W}_i]_{xy} \\
(只使用xy维度的误差,&是因为轮速只有这两个方向有观测)
\end{aligned}
$$
$$
\begin{aligned}
&变量说明: \\
&v_{li}: i时刻左后轮速测量值 \\
&v_{ri}: i时刻右后轮速测量值 \\
&v^{W}_i: i时刻后轴中心(轮子坐标系的原点)速度测量值（轮子坐标系） \\
&\hat{v}^{W}_i: i时刻后轴中心速度的估计值(轮子坐标系) \\ 
&{\omega}_i: i时刻imu的旋转角速度(Imu系) \\
&R_{wI_i}: i时刻Imu在世界系下姿态\\
&v^{w}_i: i时刻后轴中心速度的估计值(世界坐标系) \\
&bg_i: i时刻bg的估计值 \\
&R_{IW}, t_{IW}: 轮子系和Imu系的外参
\end{aligned}
$$
## Wheel Velocity Factor Jacobian推导
$$
\begin{aligned}
r_v^{'} 
&= v^{W}_i - \hat{v}^{W}_i \\
\frac{\partial{r_v^{'}}}{\partial{p_{wI_i}}}
&= 0 \\
\frac{\partial{r_v^{'}}}{\partial{q_{wI_i}}}
&= \frac{\partial{r_v^{'}}}{\partial{\hat{v}^{W}_i}}\frac{\partial{\hat{v}^{W}_i}}{\partial{q_{wI_i}}} \\
&= -I * R_{IW}^T[R_{wI_i}^Tv^{w}_{i}]_{\times} \\
&= -R_{IW}^T[R_{wI_i}^Tv^{w}_{i}]_{\times} \\
\frac{\partial{r_v^{'}}}{\partial{v^{w}_i}}
&= -R_{IW}^TR_{wI_i}^T \\
\frac{\partial{r_v^{'}}}{\partial{ba_{i}}}
&= 0 \\
\frac{\partial{r_v^{'}}}{\partial{bg_{i}}}
&= -R_{IW}^{T}[t_{IW}]_{\times} \\
\frac{\partial{r_v^{'}}}{\partial{R_{IW}}}
&= R_{IW}^{T}[R_{wI_i}^Tv^{w}_{i} + [\omega_i - bg_i]_{\times}t_{IW}]_{\times} \\
\frac{\partial{r_v^{'}}}{\partial{t_{IW}}}
&= -R_{IW}^T[{\omega}_i - bg_i]_{\times}
\end{aligned}
$$