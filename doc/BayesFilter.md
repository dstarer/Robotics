## 贝叶斯滤波算法

### g-h 滤波框架

```
初始化
	1. 初始化状态
	2. 初始化状态初值的置信度，通常为设置协方差矩阵

预测
	1. 根据模型预测下一个状态
	2. 调整状态的置信度，通常为更新协方差矩阵

更新
	1. 获取观测数据及观测数据的置信度(协方差矩阵)
	2. 计算当前状态与观测状态的残差。
	3. 依据观测置信度，更新当前的状态，及置信度。
```

### 贝叶斯滤波推导

**符号约定**

1. $ x$通常表示状态。
2. $Z$通常表示观测向量，实际由m维向量组成，$z_i$表示第i个观测向量。
3. $u$表示动作。

贝叶斯滤波是g-h滤波的一类算法，也是卡尔曼滤波，粒子滤波等的基础，这里只是阐述其推导过程。

**基础知识**

**贝叶斯公式**
$$
P(x,  z) = P( x|z) P(z) = P(z|x) P(x)
$$

$$
P(x|z) =\frac {P(z|x)P(x) }{P(z)}
$$

1. $P(z|x)$通常是预先知道的，也就是在状态为$x$的情况下，观测到$z$的概率。
2. $P(x|z)$是通过观测到$z$，推断实际值是$x$的概率。
3. $P(x), P(z)$是先验知识

由于$z$通常是我们观测到的，$P(x|z)$无关，所以也可以写作：
$$
P(x|z) = \eta P(z|x) P(x)
\\ \eta = \frac {1} {P(z)}
$$
$P(z)$用全概率公式展开有：
$$
P(z) = \int P(z|x)P(x) dx
$$
**多观测值融合**

前面是对于单一观测值下，对状态的估计。而实际中会是依据多观测值估计当前状态，也就是要计算$P(x|Z)$
$$
\begin{split}
P(x|Z) &= P(x|z_1, z_2, \cdots , z_m) \\ 
&= \frac {P(x, z_1, z_2, \cdots, z_m) } {P(Z)} \\
&=  \frac {P(z_1| x, z_2, \cdots, z_m) P(x, z_2, \cdots, z_m)} {P(Z)} \\
&= \frac{P(z_1| x, z_2, \cdots, z_m)P(z_2 | x, z_3, \cdots, z_m) P(x, z_3, \cdots ,z_m)}{P(Z)} \\
&= \frac{P(z_1| x, z_2, \cdots, z_m) P(z_2 | x, z_3, \cdots , z_m) \cdots P(z_m|x) P(x)} {P(Z)}
\end{split} \tag{5}
$$
通常假设各观测量之间彼此独立，也即$z_i$只与$x$有关，所以有
$$
\begin{split}
P(x| Z) &= \frac {P(z_1|x) P(z_2|x) \cdots P(z_m|x) P(x)}  {P(Z)} \\
&=\frac {P(z_1|x) P(z_2|x) \cdots P(z_m|x) P(x)} {\prod_{i=1}^m P(z_i)} \\
&= \prod_{i=1}^m \eta_i \prod_{j=1}^m P(z_j|x)P(x)
\end{split} \tag{6}
$$
**马尔可夫过程**

**贝叶斯滤波算法**

**问题描述**

已知机器人初始状态为$x_0$，假设t-1时刻状态为$x_{t-1}$，当前输入为$u_t$，得到状态$x_t$，此时机器人观测量为$z_t$，求解某一时刻机器人的状态$x_t$。

输入：

   	1. 1 到t时刻的状态动作和观测值：$u_1, z_1, \cdots, u_t, z_t$
   	2. 观测模型：$P(z|x)$
   	3. 状态转移模型：$P(x_t| x_{t-1}, u_t)$
   	4. 系统状态先验分布：$P(x)$

输出：

$Bel(x_t) = P(x_t| x_0, u_1, z_1, ..., u_t, z_t)$

**算法假设**

1. 遵循马尔可夫假设：$x_t$仅由$x_{t-1}$和$u_t$决定，当$x_t$确定时，观测量$z_t$仅与$x_t$有关；
2. 静态环境，环境不发生动态变化；
3. 观测噪声和模型噪声相互独立。

推导过程如下：
$$
\begin{split}
Bel(x_t)&=P(x_t|x_0, u_1, z_1, \cdots, u_t, z_t) \\
&= \frac {P(z_t | x_t, x_0, u_1, z_1, \cdots,  u_t) P(x_t, x_0, u_1, z_1, \cdots, u_t)} {P(x_0, u_1, z_1, \cdots, u_t, z_t)} \\
&= \frac {P(z_t|x_t) P(x_t, x_0, u_1, z_1, \cdots, u_t)} {P(x_0, u_1, z_1, \cdots, u_t, z_t)} \\
&= \frac {P(z_t|x_t) \int P(x_t, x_0, u_1, z_1, \cdots, u_t, x_{t-1}) dx_{t-1} } {P(x_0, u_1, z_1, \cdots, u_t, z_t)} \\
&= \frac {P(z_t|x_t) \int P(x_t | x_0, u_1, z_1, \cdots, u_t, x_{t-1}) P(x_0, u_1, z_1,\cdots, u_t, x_{t-1}) dx_{t-1}} {P(x_0, u_1, z_1, \cdots, u_t, z_t)} \\
&= \frac {P(z_t|x_t) \int P(x_t| x_{t-1}, u_{t}) P(x_{t-1}| x_0, u_1, z_1, \cdots, u_t) P(x_0, u_1, z_1, \cdots, u_t) dx_{t-1} } {P(x_0, u_1, z_1, \cdots, u_t, z_t)} \\
&= \frac {P(z_t|x_t) P(x_0, u_1, z_1, \cdots, u_t) \int P(x_t| x_{t-1}, u_t) P(x_{t-1}| x_0, u_1, z_1, \cdots, u_t) d x_{t_1}} { P(x_0, u_1, z_1, \cdots, u_t, z_t) }\\
&= \frac {P(z_t|x_t) P(x_0, u_1, z_1, \cdots, u_t) \int P(x_t| x_{t-1}, u_t) P(x_{t-1} | x_0, u_1, z_1, \cdots, z_{t-1}) dx_{t-1} } { P(x_0, u_1, z_1, \cdots, u_t, z_t) } \\
&= \frac {P(z_t|x_t) P(x_0, u_1, z_1, \cdots, u_t) \int P(x_t| x_{t-1}, u_t) Bel(x_{t-1}) dx_{t-1}} { P(x_0, u_1, z_1, \cdots, u_t, z_t)} \\
&= \frac {P(z_t|x_t) \int P(x_t| x_{t-1}, u_t) Bel(x_{t-1}) dx_{t-1}} {P(z_t | x_0, u_1, z_1, \cdots, u_t)} \\
&= \eta_{z_t} P(z_t|x_t) \int P(x_t|x_{t-1}, u_t)Bel(x_{t-1}) dx_{t-1}
\end{split} \tag{7}
$$

推导过程中，

第一步是贝叶斯概率公式；

第二步运用马尔可夫假设；

第三步运用全概率公式；

第四步应用全概率公式；

第五步应用马尔可夫假设；

第六步$P(x_0, u_1, z_1, \cdots, u_t)与x_{t-1}$无关，视为常量，提取到积分外；

第七步$x_{t-1}$与$u_t$无关，运用马尔可夫假设；

第九步，全概率公式，消除$P(x_0, u_1, z_1, \cdots, u_t)$

根据递推式，有

预测：
$$
\int P(x_t|x_{t-1}, u_t) Bel(x_{t-1}) dx_{t-1} \tag{8}
$$
更新：
$$
\eta_{z_t} P(z_t | x_t) \tag{9}
$$
算法流程：

​	BayesFilter(Bel(x), d):

​                $\eta = 0$

​		如果d是观测数据， 则

​			for all x do:

​                              $Bel(x) = P(z|x) Bel(x)$

​                              $\eta = \eta + Bel(x)$

​		       for all x do

​                             $Bel(x) = \eta^{-1} Bel(x)$

​		如果d是动作，则

​		       for all x do:

​                             $Bel(x) = \int P(x|x',u)Bel(x')dx'$

### reference

1.  [Kalman-and-Bayesian-Filters-in-Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/01-g-h-filter.ipynb)
2. [细说贝叶斯滤波](https://www.cnblogs.com/ycwang16/p/5995702.html)
3. 