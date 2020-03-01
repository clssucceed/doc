
[TOC]
# 基础理论
## IMU测量模型(Sensor Model)
* $\widetilde{\omega}^b = {\omega}^b + bg + ng$
* $\widetilde{a}^b = q_{bw}(a^w + G^w) + ba + na$
* $\omega$, $a$表示真值，$\widetilde{\omega}$, $\widetilde{a}$表示测量值
* $bg$, $ng$, $ba$, $na$为了下面推导方便都省略了上标b(body)
* 除了上标中的b表示body，其他字母分别对应如下含义
  b: bias
  n: noise
  g: gyr
  a: acc
  w: world
  G: Gravity
## 运动学模型(Kinetic Model)
* $\dot{q}_{wb_t} = q_{wb_t} \otimes \begin{bmatrix} 0 \\ 0.5\omega^{b_t} \end{bmatrix}$
* $\dot{p}_{wb_t} = v^w_t$
* $\dot{v}^w_t = a^w_t$
## IMU积分公式
* $q_{wb_j} = \int_{t\in[i, j]}{q_{wb_t} \otimes \begin{bmatrix} 0 \\ 0.5\omega^{b_t}\end{bmatrix}\delta{t}}$
* $p_{wb_j} = p_{wb_i} + v^w_i \Delta{t} + \iint_{t\in[i, j]}({q_{wb_t}a^{b_t} - G^w})\delta{t^2}$
* $v^w_j = v^w_i + \int_{t\in[i, j]}{(q_{wb_t}a^{b_t} - G^w)\delta{t}}$
## 预积分基本原理
* 因为所有的积分操作都和$q_{wb_t}$有关，每次优化迭代之后$q_{wb_i}$会发生变化，进而导致$q_{wb_t}$发生变化，所以积分操作都需要重新计算才可以获取j时刻的状态，进而获取residual
* 预积分就是积分中和状态量qpv无关的那部分，完整的积分就变成预积分和状态量qpv的"乘法"运算结果，这样状态量变化就不需要重新积分，计算量就会减少很多 

# Residual公式推导
## 预积分表达式推导
从上述IMU积分公式中不难发现，积分是通过$q_{wb_t}$和状态量产生关联的．所以，通过公式$q_{wb_t} = q_{wb_i} \otimes q_{b_ib_t}$就可以将状态量$q_{wb_i}$从积分中隔离出来，具体如下
* $q_{wb_j} = q_{wb_i}\int_{t\in[i, j]}{q_{b_ib_t} \otimes \begin{bmatrix} 0 \\ 0.5\omega^{b_t}\end{bmatrix}\delta{t}}$
* $p_{wb_j} = p_{wb_i} + v^w_i \Delta{t} - 0.5G^w\Delta{t^2} + q_{wb_i}\iint_{t\in[i, j]}({q_{b_ib_t}a^{b_t}})\delta{t^2}$
* $v^w_j = v^w_i - G^w\Delta{t} + q_{wb_i}\int_{t\in[i, j]}{(q_{b_ib_t}a^{b_t})\delta{t}}$

经过上述变形，不难发现积分已经和状态量完全不相关，上面公式中的积分即为预积分，具体如下所示
* $q_{b_ib_j} = \int_{t\in[i, j]}{q_{b_ib_t} \otimes \begin{bmatrix} 0 \\ 0.5\omega^{b_t}\end{bmatrix}\delta{t}}$ 
* $\alpha_{b_ib_j} = \iint_{t\in[i, j]}({q_{b_ib_t}a^{b_t}})\delta{t^2}$
* $\beta_{b_ib_j} = \int_{t\in[i, j]}{(q_{b_ib_t}a^{b_t})\delta{t}}$

预积分的物理意义可以理解为j时刻的qpv在i时刻坐标系下的表示，其中假设i时刻的q为单位四元数，pv都为0.
## 预积分误差推导
使用预积分替换掉IMU积分公式中积分部分，可获得如下等式
* $q_{wb_j} = q_{wb_i} \otimes q_{b_ib_j}$
* $p_{wb_j} = p_{wb_i} + v^w_i \Delta{t} - 0.5G^w\Delta{t^2} + q_{wb_i}\alpha_{b_ib_j}$
* $v^w_j = v^w_i - G^w\Delta{t} + q_{wb_i}\beta_{b_ib_j}$
* $b_{a_j} = b_{a_i}$
* $b_{g_j} = b_{g_i}$

对上述公式变形获取预积分的状态表达式，然后再和预积分求差即为预积分误差，具体如下所示
* $r_q = 2[q_{b_ib_j}^{*} \otimes (q_{wb_i}^* \otimes q_{wb_j})]_{xyz}$    
* $r_p = q_{wb_i}^*(p_{wb_j} - p_{wb_i} - v^w_i \Delta{t} + 0.5G^w\Delta{t^2}) - \alpha_{b_ib_j}$
* $r_v = q_{wb_i}^* (v^w_j - v^w_i + G^w\Delta{t}) - \beta_{b_ib_j}$
* $r_{ba} = ba_j - ba_i$
* $r_{bg} = bg_j - bg_i$

# qpvbabg的递推公式推导
qpv推导使用的是mid-point方法，即两个相邻时刻k和k+1的预积分增量使用两个时刻的测量值的平均值来计算．此处的推导不考虑噪声
* $\omega = 0.5(({\omega}^{b_k} - bg_k) + ({\omega}^{b_{k+1}} - bg_{k}))$
* $q_{b_ib_{k+1}} = q_{b_ib_k} \otimes \begin{bmatrix} 1 \\ 0.5{\omega}\delta{t}\end{bmatrix}$
* $a = 0.5(q_{b_ib_k}(a^{b_k} - ba_k) + q_{b_ib_{k+1}}(a^{b_{k+1}} - ba_k))$
* ${\alpha}_{b_ib_{k+1}} = {\alpha}_{b_ib_k} + {\beta}_{b_ib_k}\delta{t} + 0.5a\delta{t^2}$
* ${\beta}_{b_ib_{k+1}} = {\beta}_{b_ib_k} + a\delta{t}$
* $ba_{k+1} = ba_k + n_{ba_k}\delta{t}$
* $bg_{k+1} = bg_k + n_{bg_k}\delta{t}$
* ba,bg推导使用的是随机游走模型，即变化率符合高斯分布．$n_{ba}$, $n_{bg}$分别表示ba,bg的随机游走参数．

# Jacobian和Covariance的递推公式推导
## 基础理论
### Covairance传递公式
* $x_{k+1} = F_kx_k + G_kn_k$
* ${\Sigma}_{k+1} = F_k{\Sigma}_{k}F_k^T + G_k{\Sigma}_nG_k^T$
* $\Sigma_0 = 0$
### Jacobian传递公式
* $x_{k+1} = F_kx_k + G_kn_k$
* $J_{k+1} = F_kJ_k$
* $J_0 = I$

### F/G计算基本思想
* error_state法: 将上述qpvbabg递推公式中预积分，状态，噪声替换成normal_state + error_state的格式，然后变形获取需要的两个error_state的商即可
* 直接求导法: 利用链式求导法则和常用的求导结果获取所需结果即可．
* error_state法物理意义更加明确，但是推导会复杂一些；直接求导法需要对相关数学知识比较熟悉，但是推导会简单一些
* 下面的推导会根据具体情况交替使用两种方法

## 具体推导
上述qpvbabg公式考虑进噪声如下所示
* $\omega = 0.5(({\omega}^{b_k} - bg_k - ng_k) + ({\omega}^{b_{k+1}} - bg_{k} - ng_{k+1)})$
* $q_{b_ib_{k+1}} = q_{b_ib_k} \otimes \begin{bmatrix} 1 \\ 0.5{\omega}\delta{t}\end{bmatrix}$
* $a = 0.5(q_{b_ib_k}(a^{b_k} - ba_k - na_k) + q_{b_ib_{k+1}}(a^{b_{k+1}} - ba_k - na_{k+1}))$
* ${\alpha}_{b_ib_{k+1}} = {\alpha}_{b_ib_k} + {\beta}_{b_ib_k}\delta{t} + 0.5a\delta{t^2}$
* ${\beta}_{b_ib_{k+1}} = {\beta}_{b_ib_k} + a\delta{t}$
* $ba_{k+1} = ba_k + n_{ba_k}\delta{t}$
* $bg_{k+1} = bg_k + n_{bg_k}\delta{t}$

F/G的定义如下所示
* $\begin{bmatrix} \delta{{\alpha}_{b_ib_{k+1}}} \\ \delta{{\theta}_{b_ib_{k+1}}} \\ \delta{{\beta}_{b_ib_{k+1}}} \\ \delta{{ba}_{k+1}} \\ \delta{{bg}_{k+1}}\end{bmatrix} = F\begin{bmatrix} \delta{{\alpha}_{b_ib_{k}}} \\ \delta{{\theta}_{b_ib_{k}}} \\ \delta{{\beta}_{b_ib_{k}}} \\ \delta{{ba}_{k}} \\ \delta{{bg}_{k}}\end{bmatrix} + G\begin{bmatrix} na_{k} \\ ng_{k} \\ na_{k+1} \\ ng_{k+1} \\ n_{ba_k} \\ n_{bg_k}\end{bmatrix}$ 
* $\delta{x}_{k+1} = F\delta{x}_{k} + Gn_k$

### 计算F
#### 计算alpha的导数$\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\delta{x}_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\delta{\alpha}_{b_ib_k}}} &= I \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\delta{\theta}_{b_ib_k}}} &= 0.5\delta{t^2}\color{#f00}{\frac{\partial{a}}{\partial{\delta{\theta}_{b_ib_k}}}} \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\delta{\beta}_{b_ib_k}}} &= \delta{t}I \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\delta{ba}_{k}}} &= 0.5\delta{t^2}\color{#f00}{\frac{\partial{a}}{\partial{\delta{ba}_{b_k}}}} \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\delta{bg}_{k}}} &= 0.5\delta{t^2}\color{#f00}{\frac{\partial{a}}{\partial{\delta{bg}_{b_k}}}} \\
\end{aligned}
$$ 
#### 计算theta的导数$\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{x}_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{\alpha}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{\theta}_{b_ib_k}}} &= I - \hat{\omega}\delta{t} \color{#f00}{({error\_state法})}\\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{\beta}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{ba}_{k}}} &= 0 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{bg}_{k}}} &= \frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{\omega}}}\frac{\partial{\delta{\omega}}}{\partial{\delta{bg}_k}} \\
&= \delta{t}I * (-1) \color{#f00}{(\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{\omega}}}使用error\_state法)}
\end{aligned}
$$ 
#### 计算beta的导数$\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{\delta{x}_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{\delta{\alpha}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{\delta{\theta}_{b_ib_k}}} &= \delta{t}\color{#f00}{\frac{\partial{a}}{\partial{\delta{\theta}_{b_ib_k}}}} \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{\delta{\beta}_{b_ib_k}}} &= I \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{\delta{ba}_{k}}} &= \delta{t}\color{#f00}{\frac{\partial{a}}{\partial{\delta{ba}_{b_k}}}} \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{\delta{bg}_{k}}} &= \delta{t}\color{#f00}{\frac{\partial{a}}{\partial{\delta{bg}_{b_k}}}} \\
\end{aligned}
$$ 
#### 计算ba的导数$\frac{\partial{\delta{ba}_{k+1}}}{\partial{\delta{x}_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{ba}_{k+1}}}{\partial{\delta{\alpha}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{\delta{\theta}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{\delta{\beta}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{\delta{ba}_{k}}} &= I \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{\delta{bg}_{k}}} &= 0 \\
\end{aligned}
$$ 
#### 计算bg的导数$\frac{\partial{\delta{bg}_{k+1}}}{\partial{\delta{x}_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{bg}_{k+1}}}{\partial{\delta{\alpha}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{\delta{\theta}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{\delta{\beta}_{b_ib_k}}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{\delta{ba}_{k}}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{\delta{bg}_{k}}} &= I \\
\end{aligned}
$$

### 计算G
#### 计算alpha的导数$\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{n_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{na_k}} &= \frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{na_k}} \\
&= 0.5\delta{t^2} * 0.5q_{b_ib_k} \\
&= 0.25q_{b_ib_k}\delta{t^2} \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{ng_k}} &= \frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{ng_k}} \\
&= 0.5\delta{t^2}\color{#f00}{\frac{\partial{a}}{\partial{ng_k}}} \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{na_{k+1}}} &= \frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{na_{k+1}}} \\
&= 0.5\delta{t^2} * 0.5q_{b_ib_{k+1}} \\
&= 0.25q_{b_ib_{k+1}}\delta{t^2} \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{ng_{k+1}}} &= \frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{ng_{k+1}}} \\
&= 0.5\delta{t^2}\color{#f00}{\frac{\partial{a}}{\partial{ng_{k+1}}}} \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{n_{ba_{k}}}} &= 0 \\
\frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{n_{bg_{k}}}} &= 0 \\
\end{aligned}
$$
#### 计算theta的导数$\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{n_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{na_k}} &= 0 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{ng_k}} &= \frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\omega}}\frac{\partial{\omega}}{\partial{ng_k}} \\
&= \delta{t}I * 0.5 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{na_{k+1}}} &= 0 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{ng_{k+1}}} &= \frac{\partial{\delta{\alpha}_{b_ib_{k+1}}}}{\partial{\omega}}\frac{\partial{\omega}}{\partial{ng_{k+1}}} \\
&= \delta{t}I * 0.5 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{n_{ba_{k}}}} &= 0 \\
\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{n_{bg_{k}}}} &= 0 \\
\end{aligned}
$$
#### 计算beta的导数$\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{n_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{na_k}} &= \frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{na_k}} \\
&= \delta{t} * 0.5q_{b_ib_k} \\
&= 0.5q_{b_ib_k}\delta{t} \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{ng_k}} &= \frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{ng_k}} \\
&= \delta{t}\color{#f00}{\frac{\partial{a}}{\partial{ng_k}}} \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{na_{k+1}}} &= \frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{na_{k+1}}} \\
&= \delta{t} * 0.5q_{b_ib_{k+1}} \\
&= 0.5q_{b_ib_{k+1}}\delta{t} \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{ng_{k+1}}} &= \frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{a}}\frac{\partial{a}}{\partial{ng_{k+1}}} \\
&= \delta{t}\color{#f00}{\frac{\partial{a}}{\partial{ng_{k+1}}}} \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{n_{ba_{k}}}} &= 0 \\
\frac{\partial{\delta{\beta}_{b_ib_{k+1}}}}{\partial{n_{bg_{k}}}} &= 0 \\
\end{aligned}
$$
#### 计算ba的导数$\frac{\partial{\delta{ba}_{k+1}}}{\partial{n_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{ba}_{k+1}}}{\partial{na_k}} &= 0 \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{ng_k}} &= 0 \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{na_{k+1}}} &= 0 \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{ng_{k+1}}} &= 0 \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{n_{ba_{k}}}} &= \delta{t}I \\
\frac{\partial{\delta{ba}_{k+1}}}{\partial{n_{bg_{k}}}} &= 0 \\
\end{aligned}
$$
#### 计算bg的导数$\frac{\partial{\delta{bg}_{k+1}}}{\partial{n_k}}$
$$
\begin{aligned}
\frac{\partial{\delta{bg}_{k+1}}}{\partial{na_k}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{ng_k}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{na_{k+1}}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{ng_{k+1}}} &= 0 \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{n_{ba_{k}}}} &= \delta{t}I \\
\frac{\partial{\delta{bg}_{k+1}}}{\partial{n_{bg_{k}}}} &= 0 \\
\end{aligned}
$$

### 计算a的导数
由于a的表达式涉及的变量较多，且存在多层较为复杂的复合函数，所以其相关的导数计算较为复杂，上面的推导中都没有给出结果以及推导过程，只是用红色标记了出来．此外，有些导数被多次用到，先不将a的导数展开，更加容易看出F/G中的不同元素之间的关系
#### a的表达式
* $\omega = 0.5(({\omega}^{b_k} - bg_k - ng_k) + ({\omega}^{b_{k+1}} - bg_{k} - ng_{k+1})) = 0.5({\omega}_k + {\omega}_{k+1})$
* $q_{b_ib_{k+1}} = q_{b_ib_k} \otimes \begin{bmatrix} 1 \\ 0.5{\omega}\delta{t}\end{bmatrix}$
* $a = 0.5(q_{b_ib_k}(a^{b_k} - ba_k - na_k) + q_{b_ib_{k+1}}(a^{b_{k+1}} - ba_k - na_{k+1})) = 0.5(a_k + a_{k+1})$
#### 具体推导
* 
$$
\begin{aligned}
\frac{\partial{a}}{\partial{{\theta}_{b_ib_k}}}
&= 0.5(\frac{\partial{a_k}}{\partial{{\theta}_{b_ib_k}}} + \frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_k}}}) \\
\frac{\partial{a_k}}{\partial{{\theta}_{b_ib_k}}}
&= \frac{\partial{R_{b_ib_k}\hat{\theta}_{b_ib_k}(a^{b_k} - ba_k)}}{\partial{{\theta}_{b_ib_k}}}\\
&= -R_{b_ib_{k}}(a^{b_{k}} - ba_k)_{\times} \color{#f00}{(常用求导结果)} \\ 
\frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_k}}}
&= -R_{b_ib_{k}}(q_{b_kb_{k+1}}(a^{b_{k+1}} - ba_k))_{\times} \\
&= -R_{b_ib_{k}}R_{b_kb_{k+1}}(a^{b_{k+1}} - ba_k)_{\times}R_{b_kb_{k+1}}^{T} \\
&= -R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_k)_{\times}(I - \hat{\omega}\delta{t}) \\
\frac{\partial{a}}{\partial{{\theta}_{b_ib_k}}}
&= -0.5(R_{b_ib_{k}}(a^{b_{k}} - ba_k)_{\times} + R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_k)_{\times}(I - \hat{\omega}\delta{t})) \\
\end{aligned}
$$
* 
$$
\begin{aligned}
\frac{\partial{a}}{\partial{ba_k}}
&= -0.5(R_{b_ib_k} + R_{b_ib_{k+1}})
\end{aligned}
$$
* 
$$
\begin{aligned}
\frac{\partial{a}}{\partial{bg_k}}
&= 0.5\frac{\partial{a_{k+1}}}{\partial{bg_k}} \\
&= 0.5\frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_{k+1}}}}\frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{bg_k}} \\
\frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_{k+1}}}}
&= -R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times} 
\color{#f00}{(常用求导结果)}\\
\frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{bg_k}}
&= \frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{\omega}}\frac{\partial{\omega}}{\partial{bg_k}} \\
&= \delta{t}I*(-1) \color{#f00}{(\frac{\partial{q_{b_ib_{k+1}}}}{\partial{\omega}}使用error\_state法)} \\
\frac{\partial{a}}{\partial{bg_k}}
&= 0.5\delta{t}R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times} \\
\end{aligned}
$$
* 
$$
\begin{aligned}
\frac{\partial{a}}{\partial{ng_k}}
&= 0.5\frac{\partial{a_{k+1}}}{\partial{ng_k}} \\
&= 0.5\frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_{k+1}}}}\frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{ng_k}} \\
\frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_{k+1}}}}
&= -R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times} 
\color{#f00}{(常用求导结果)}\\
\frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{ng_k}}
&= \frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{\omega}}\frac{\partial{\omega}}{\partial{ng_k}} \\
&= \delta{t}I*(-0.5) \\
\frac{\partial{a}}{\partial{ng_k}}
&= 0.25\delta{t}R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times} \\
\end{aligned}
$$
* 
$$
\begin{aligned}
\frac{\partial{a}}{\partial{ng_{k+1}}}
&= 0.5\frac{\partial{a_{k+1}}}{\partial{ng_{k+1}}} \\
&= 0.5\frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_{k+1}}}}\frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{ng_{k+1}}} \\
\frac{\partial{a_{k+1}}}{\partial{{\theta}_{b_ib_{k+1}}}}
&= -R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times} 
\color{#f00}{(常用求导结果)}\\
\frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{ng_{k+1}}}
&= \frac{\partial{{\theta}_{b_ib_{k+1}}}}{\partial{\omega}}\frac{\partial{\omega}}{\partial{ng_{k+1}}} \\
&= \delta{t}I*(-0.5) \\
\frac{\partial{a}}{\partial{ng_{k+1}}}
&= 0.25\delta{t}R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times} \\
\end{aligned}
$$

### 使用error_state法推导的导数的推导过程
#### 证明$\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{\theta}_{b_ib_k}}} = I - \hat{\omega}\delta{t}$
$$
\begin{aligned}
q_{b_ib_{k+1}} &= q_{b_ib_k} \otimes \begin{bmatrix} 1 \\ 0.5{\omega}\delta{t}\end{bmatrix} \\
q_{b_ib_{k+1}}(I + \hat{\delta{\theta}}_{b_ib_{k+1}}) &= q_{b_ib_k}(I + \hat{\delta{\theta}}_{b_ib_{k}}) (I + \hat{\omega}\delta{t}) \\
q_{b_ib_{k+1}}\hat{\delta{\theta}}_{b_ib_{k+1}} &= q_{b_ib_k}\hat{\delta{\theta}}_{b_ib_{k}} (I + \hat{\omega}\delta{t}) \\
\hat{\delta{\theta}}_{b_ib_{k+1}} &= q_{b_ib_{k+1}}^{*}q_{b_ib_k}\hat{\delta{\theta}}_{b_ib_{k}} (I + \hat{\omega}\delta{t}) \\
\hat{\delta{\theta}}_{b_ib_{k+1}} &= (I - \hat{\omega}\delta{t})\hat{\delta{\theta}}_{b_ib_{k}} (I + \hat{\omega}\delta{t}) \\
\hat{\delta{\theta}}_{b_ib_{k+1}} &=\widehat{ (I - \hat{\omega}\delta{t})\delta{\theta}}_{b_ib_{k}} \\
\delta{\theta}_{b_ib_{k+1}} &=(I - \hat{\omega}\delta{t})\delta{\theta}_{b_ib_{k}} \\
\frac{\delta{\theta}_{b_ib_{k+1}}}{\delta{\theta}_{b_ib_{k}}} &= I - \hat{\omega}\delta{t}
\end{aligned}
$$
#### 证明$\frac{\partial{\delta{\theta}_{b_ib_{k+1}}}}{\partial{\delta{\omega}}} = \delta{t}I$
$$
\begin{aligned}
q_{b_ib_{k+1}} &= q_{b_ib_k} \otimes \begin{bmatrix} 1 \\ 0.5{\omega}\delta{t}\end{bmatrix} \\
q_{b_ib_{k+1}}(I + \hat{\delta{\theta}}_{b_ib_{k+1}}) &= q_{b_ib_k} (I + \hat{\omega}\delta{t})(I + \hat{\delta{\omega}}\delta{t}) \\
q_{b_ib_{k+1}}\hat{\delta{\theta}}_{b_ib_{k+1}} &= q_{b_ib_k} (I + \hat{\omega}\delta{t})\hat{\delta{\omega}}\delta{t} \\
\hat{\delta{\theta}}_{b_ib_{k+1}} &= \hat{\delta{\omega}}\delta{t} \\
\delta{\theta}_{b_ib_{k+1}} &= \delta{\omega}\delta{t} \\
\frac{\delta{\theta}_{b_ib_{k+1}}}{\delta{\omega}} &= \delta{t}I
\end{aligned}
$$

### F/G完整结果
* $\begin{bmatrix} \delta{{\alpha}_{b_ib_{k+1}}} \\ \delta{{\theta}_{b_ib_{k+1}}} \\ \delta{{\beta}_{b_ib_{k+1}}} \\ \delta{{ba}_{k+1}} \\ \delta{{bg}_{k+1}}\end{bmatrix} = F\begin{bmatrix} \delta{{\alpha}_{b_ib_{k}}} \\ \delta{{\theta}_{b_ib_{k}}} \\ \delta{{\beta}_{b_ib_{k}}} \\ \delta{{ba}_{k}} \\ \delta{{bg}_{k}}\end{bmatrix} + G\begin{bmatrix} na_{k} \\ ng_{k} \\ na_{k+1} \\ ng_{k+1} \\ n_{ba_k} \\ n_{bg_k}\end{bmatrix}$ 
* $\delta{x}_{k+1} = F\delta{x}_{k} + Gn_k$
* 
$$
F = \begin{aligned}
\begin{bmatrix}
&I 
&0.5\delta{t^2}\frac{\partial{a}}{\partial{\theta_{b_ib_k}}} 
&\delta{t}I 
&0.5\delta{t^2}\frac{\partial{a}}{\partial{ba_k}}
&0.5\delta{t^2}\frac{\partial{a}}{\partial{bg_k}} \\
&0
&I - \hat{\omega}\delta{t}
&0
&0
&-\delta{t}I \\
&0
&\delta{t}\frac{\partial{a}}{\partial{\theta_{b_ib_k}}}
&I
&\delta{t}\frac{\partial{a}}{\partial{ba_k}}
&\delta{t}\frac{\partial{a}}{\partial{bg_k}} \\
&0
&0
&0
&I
&0 \\
&0
&0
&0
&0
&I
\end{bmatrix}
\end{aligned}
$$
* 
$$
G = \begin{aligned}
\begin{bmatrix}
&0.25\delta{t^2}R_{b_ib_k}
&\color{#ff0}{0.5\delta{t^2}\frac{\partial{a}}{\partial{ng_k}}}
&0.25\delta{t^2}R_{b_ib_{k+1}}
&\color{#ff0}{0.5\delta{t^2}\frac{\partial{a}}{\partial{ng_{k+1}}}}
&0
&0 \\
&0
&0.5\delta{t}I
&0
&0.5\delta{t}I
&0
&0 \\
&0.5\delta{t}R_{b_ib_k}
&\color{#ff0}{\delta{t}\frac{\partial{a}}{\partial{ng_k}}}
&0.5\delta{t}R_{b_ib_{k+1}}
&\color{#ff0}{\delta{t}\frac{\partial{a}}{\partial{ng_{k+1}}}}
&0
&0 \\
&0
&0
&0
&0
&\delta{t}I
&0 \\
&0
&0
&0
&0
&0
&\delta{t}I
\end{bmatrix}
\end{aligned} 
$$
* a的相关导数
$$
\begin{aligned}
\frac{\partial{a}}{\partial{{\theta}_{b_ib_k}}}
&= -0.5(R_{b_ib_{k}}(a^{b_{k}} - ba_k)_{\times} + R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_k)_{\times}(I - \hat{\omega}\delta{t})) \\
\frac{\partial{a}}{\partial{ba_k}}
&= -0.5(R_{b_ib_k} + R_{b_ib_{k+1}}) \\
\frac{\partial{a}}{\partial{bg_k}}
&= 0.5\delta{t}R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times} \\
\frac{\partial{a}}{\partial{ng_k}}
&= \color{#ff0}{0.25\delta{t}R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times}} \\
\frac{\partial{a}}{\partial{ng_{k+1}}}
&= \color{#ff0}{0.25\delta{t}R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times}} \\
\end{aligned}
$$
不难发现: 

1. $\frac{\partial{a}}{\partial{ng_k}} = \frac{\partial{a}}{\partial{ng_{k+1}}} = 0.5\frac{\partial{a}}{\partial{bg_k}}$ 
2. $R_{b_ib_{k+1}}(a^{b_{k+1}} - ba_{k})_{\times}$出现了4次
3. 利用上述两个规律，可以加速F/G的计算 
 
# ba/bg变化较小时预积分的更新方式推导
预积分虽然和qpv不相关，但是和ba,bg相关．不过ba,bg一般变化较慢，所以当ba,bg较小时，就将预积分在初始ba,bg处进行线性化.预积分关于ba,bg的更新方式具体如下所示:
* ba, bg变化较小:
$$
\begin{aligned}
{\alpha}^{'} &= {\alpha} + \frac{\partial{\alpha}}{\partial{ba}}\delta{ba} + \frac{\partial{\alpha}}{\partial{bg}}\delta{bg} \\
{\beta}^{'} &= {\beta} + \frac{\partial{\beta}}{\partial{ba}}\delta{ba} + \frac{\partial{\beta}}{\partial{bg}}\delta{bg} \\
{q}^{'} &= {q} * \begin{bmatrix} 1 \\ 0.5\frac{\partial{\theta}}{\partial{bg}}\delta{bg} \end{bmatrix}
\end{aligned}
$$
* ba, bg变化较大: 重新进行一次预积分

# ImuFactor的Residual和Jacobian
## Residual
* $r_p = q_{wb_i}^*(p_{wb_j} - p_{wb_i} - v^w_i \Delta{t} + 0.5G^w\Delta{t^2}) - \alpha_{b_ib_j}$
* $r_q = 2[q_{b_ib_j}^{*} \otimes (q_{wb_i}^* \otimes q_{wb_j})]_{xyz}$    
* $r_v = q_{wb_i}^* (v^w_j - v^w_i + G^w\Delta{t}) - \beta_{b_ib_j}$
* $r_{ba} = ba_j - ba_i$
* $r_{bg} = bg_j - bg_i$
## Jacobian
### 推导过程
下面的推导中
1. state=(p, q, v, ba, bg)
2. $q_1 \otimes q_2 = [q_1]_Lq_2$
3. $q_1 \otimes q_2 = [q_2]_Rq_1$
4. 需要注意的是$\alpha_{b_ib_j}$, $\beta_{b_ib_j}$, $q_{b_ib_j}$是使用新传入的ba,bg修正过的
* $\frac{\partial{r_p}}{\partial{state_i}}$
$$
\begin{aligned}
\frac{\partial{r_p}}{\partial{p_{wb_i}}}
&= -R_{wb_i}^{T} \\
\frac{\partial{r_p}}{\partial{q_{wb_i}}}
&= [R_{wb_i}^T(p_{wb_j} - p_{wb_i} - v^w_i \Delta{t} + 0.5G^w\Delta{t^2})]_\times \\
\frac{\partial{r_p}}{\partial{v_{wb_i}}}
&= -R_{wb_i}^{T}\Delta{t} \\
\frac{\partial{r_p}}{\partial{ba_i}}
&= -\frac{\partial{{\alpha}_{b_ib_j}}}{\partial{ba_i}}
(即预积分position部分关于ba_i的导数) \\
\frac{\partial{r_p}}{\partial{bg_i}}
&= -\frac{\partial{{\alpha}_{b_ib_j}}}{\partial{bg_i}}
(即预积分position部分关于bg_i的导数) \\
\end{aligned}
$$
* $\frac{\partial{r_p}}{\partial{state_j}}$
$$
\begin{aligned}
\frac{\partial{r_p}}{\partial{p_{wb_j}}}
&= R_{wb_i}^{T} \\
\frac{\partial{r_p}}{\partial{q_{wb_j}}}
&= 0 \\
\frac{\partial{r_p}}{\partial{v_{wb_j}}}
&= 0 \\
\frac{\partial{r_p}}{\partial{ba_j}}
&= 0 \\
\frac{\partial{r_p}}{\partial{bg_j}}
&= 0 \\
\end{aligned}
$$
* $\frac{\partial{r_q}}{\partial{state_i}}$
$$
\begin{aligned}
\frac{\partial{r_q}}{\partial{p_{wb_i}}}
&= 0 \\
\frac{\partial{r_q}}{\partial{q_{wb_i}}}
&= \frac{\delta{r_q}}{\delta{\theta}} \\ 
\delta{r_q} 
&= r_q^{'} - r_q \\
&= 2[q_{b_ib_j}^{*} \otimes ((q_{wb_i} \otimes \begin{bmatrix}1 \\ 0.5\delta{\theta}\end{bmatrix})^* \otimes q_{wb_j})]_{xyz} - 2[q_{b_ib_j}^{*} \otimes (q_{wb_i}^* \otimes q_{wb_j})]_{xyz} \\
&= 2[q_{b_ib_j}^{*} \otimes \begin{bmatrix}1 \\ 0.5\delta{\theta}\end{bmatrix}^* \otimes (q_{wb_i}^{*} \otimes q_{wb_j})]_{xyz} - 2[q_{b_ib_j}^{*} \otimes (q_{wb_i}^* \otimes q_{wb_j})]_{xyz} \\
&= 2[[q_{b_ib_j}^{*}]_L [q_{wb_i}^{*} \otimes q_{wb_j}]_R\begin{bmatrix}1 \\ 0.5\delta{\theta}\end{bmatrix}^*]_{xyz} - 2[[q_{b_ib_j}^{*}]_L [q_{wb_i}^{*} \otimes q_{wb_j}]_R\begin{bmatrix}1 \\ 0\end{bmatrix}]_{xyz} \\
&= 2[[q_{b_ib_j}^{*}]_L [q_{wb_i}^{*} \otimes q_{wb_j}]_R\begin{bmatrix}0 \\ -0.5\delta{\theta}\end{bmatrix}]_{xyz} \\
&= 2[\begin{bmatrix}A_{11}, B_{13} \\ C_{31}, D_{33}\end{bmatrix}\begin{bmatrix}0 \\ -0.5\delta{\theta}\end{bmatrix}]_{xyz}\\
&= -[\begin{bmatrix}B_{13}\delta{\theta} \\ D_{33}\delta{\theta}\end{bmatrix}]_{xyz}\\
&= -D_{33}\delta{\theta}\\
\frac{\partial{r_q}}{\partial{q_{wb_i}}}
&= \frac{\delta{r_q}}{\delta{\theta}} \\
&= \frac{-D_{33}\delta{\theta}}{\delta{\theta}} \\
&= -D_{33}
(D_{33}是[q_{b_ib_j}^{*}]_L [q_{wb_i}^{*} \otimes q_{wb_j}]_R矩阵的右下角的3 \times 3矩阵)\\
\frac{\partial{r_q}}{\partial{v_{wb_i}}}
&= 0 \\
\frac{\partial{r_q}}{\partial{ba_i}}
&= \frac{\partial{r_q}}{\partial{q_{b_ib_j}}}\frac{\partial{q_{b_ib_j}}}{\partial{ba_i}}\\
&= -([q_{b_ib_j}^{*} \otimes (q_{wb_i}^* \otimes q_{wb_j})]_R)_{br33}*0\\
&= 0\\
\frac{\partial{r_q}}{\partial{bg_i}}
&= \frac{\partial{r_q}}{\partial{q_{b_ib_j}}}\frac{\partial{q_{b_ib_j}}}{\partial{bg_i}}\\
&= -([q_{b_ib_j}^{*} \otimes (q_{wb_i}^* \otimes q_{wb_j})]_R)_{br33}\frac{\partial{q_{b_ib_j}}}{\partial{bg_i}}\\
A_{br33}表示A的&右下角3x3矩阵;\frac{\partial{q_{b_ib_j}}}{\partial{bg_i}}预积分quaternion部分关于bg_i的导数\\
\end{aligned}
$$
* $\frac{\partial{r_q}}{\partial{state_j}}$
$$
\begin{aligned}
\frac{\partial{r_q}}{\partial{p_{wb_j}}}
&= 0 \\
\frac{\partial{r_q}}{\partial{q_{wb_j}}}
&= [[q_{b_ib_j}^{*} \otimes q_{wb_i}^{*} \otimes q_{wb_j}]_L]_{br33} \\
\frac{\partial{r_q}}{\partial{v_{wb_i}}}
&= 0 \\
\frac{\partial{r_q}}{\partial{ba_i}}
&= 0\\ 
\frac{\partial{r_q}}{\partial{bg_i}}
&= 0\\
\end{aligned}
$$
* $\frac{\partial{r_v}}{\partial{state_i}}$
$$
\begin{aligned}
\frac{\partial{r_v}}{\partial{p_{wb_i}}}
&= 0 \\
\frac{\partial{r_v}}{\partial{q_{wb_i}}}
&= [R_{wb_i}^T(v^w_j - v^w_i + G^w\Delta{t})]_\times \\
\frac{\partial{r_v}}{\partial{v_{wb_i}}}
&= -R_{wb_i}^{T} \\
\frac{\partial{r_v}}{\partial{ba_i}}
&= -\frac{\partial{{\beta}_{b_ib_j}}}{\partial{ba_i}}
(即预积分velocity部分关于ba_i的导数) \\
\frac{\partial{r_v}}{\partial{bg_i}}
&= -\frac{\partial{{\beta}_{b_ib_j}}}{\partial{bg_i}}
(即预积分velocity部分关于bg_i的导数) \\
\end{aligned}
$$
* $\frac{\partial{r_v}}{\partial{state_j}}$
$$
\begin{aligned}
\frac{\partial{r_v}}{\partial{p_{wb_j}}}
&= 0 \\
\frac{\partial{r_v}}{\partial{q_{wb_j}}}
&= 0 \\
\frac{\partial{r_v}}{\partial{v_{wb_j}}}
&= R_{wb_i}^{T} \\
\frac{\partial{r_v}}{\partial{ba_j}}
&= 0 \\
\frac{\partial{r_v}}{\partial{bg_j}}
&= 0 \\
\end{aligned}
$$
* $\frac{\partial{r_{ba}}}{\partial{state_i}}$
$$
\begin{aligned}
\frac{\partial{r_{ba}}}{\partial{p_{wb_i}}}
&= 0 \\
\frac{\partial{r_{ba}}}{\partial{q_{wb_i}}}
&= 0 \\
\frac{\partial{r_{ba}}}{\partial{v_{wb_i}}}
&= 0 \\
\frac{\partial{r_{ba}}}{\partial{ba_i}}
&= -I \\
\frac{\partial{r_{ba}}}{\partial{bg_i}}
&= 0 \\
\end{aligned}
$$
* $\frac{\partial{r_{ba}}}{\partial{state_j}}$
$$
\begin{aligned}
\frac{\partial{r_{ba}}}{\partial{p_{wb_j}}}
&= 0 \\
\frac{\partial{r_{ba}}}{\partial{q_{wb_j}}}
&= 0 \\
\frac{\partial{r_{ba}}}{\partial{v_{wb_j}}}
&= 0 \\
\frac{\partial{r_{ba}}}{\partial{ba_j}}
&= I \\
\frac{\partial{r_{ba}}}{\partial{bg_j}}
&= 0 \\
\end{aligned}
$$
* $\frac{\partial{r_{bg}}}{\partial{state_i}}$
$$
\begin{aligned}
\frac{\partial{r_{bg}}}{\partial{p_{wb_i}}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{q_{wb_i}}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{v_{wb_i}}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{ba_i}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{bg_i}}
&= -I \\
\end{aligned}
$$
* $\frac{\partial{r_{bg}}}{\partial{state_j}}$
$$
\begin{aligned}
\frac{\partial{r_{bg}}}{\partial{p_{wb_j}}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{q_{wb_j}}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{v_{wb_j}}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{ba_j}}
&= 0 \\
\frac{\partial{r_{bg}}}{\partial{bg_j}}
&= I \\
\end{aligned}
$$
### ImuFactor完整的Jacobian
$$
J_i = 
\begin{bmatrix}
&-R_{wb_i}^{T}, 
&[R_{wb_i}^T(p_{wb_j} - p_{wb_i} - v^w_i \Delta{t} + 0.5G^w\Delta{t^2})]_\times, 
&-R_{wb_i}^{T}\Delta{t},
&-\frac{\partial{{\alpha}_{b_ib_j}}}{\partial{ba_i}},
&-\frac{\partial{{\alpha}_{b_ib_j}}}{\partial{bg_i}} \\
&0,
&\color{#ff0}{-[[q_{b_ib_j}^{*}]_L [q_{wb_i}^{*} \otimes q_{wb_j}]_R]_{br33}},
&0,
&0,
&\color{#ff0}{-([q_{b_ib_j}^{*} \otimes (q_{wb_i}^* \otimes q_{wb_j})]_R)_{br33}\frac{\partial{q_{b_ib_j}}}{\partial{bg_i}}} \\
&0,
&[R_{wb_i}^T(v^w_j - v^w_i + G^w\Delta{t})]_\times,
&-R_{wb_i}^{T},
&-\frac{\partial{{\beta}_{b_ib_j}}}{\partial{ba_i}},
&-\frac{\partial{{\beta}_{b_ib_j}}}{\partial{bg_i}} \\
&0,
&0,
&0,
&-I,
&0 \\
&0,
&0,
&0,
&0,
&-I,
\end{bmatrix} \\
J_j = 
\begin{bmatrix}
&R_{wb_i}^{T}, 
&0, 
&0,
&0,
&0 \\
&0,
&[[q_{b_ib_j}^{*} \otimes q_{wb_i}^{*} \otimes q_{wb_j}]_L]_{br33},
&0,
&0,
&0 \\
&0,
&0,
&R_{wb_i}^{T},
&0,
&0 \\
&0,
&0,
&0,
&I,
&0 \\
&0,
&0,
&0,
&0,
&I
\end{bmatrix}
$$
