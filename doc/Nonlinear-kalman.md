# 卡尔曼滤波在非线性系统下的应用

## 扩展卡尔曼滤波

主要思路：将递推公式用一阶泰勒展开近似，缺点也比较明显，某些情况下，局部线性化并不能很好的逼近真实值。

对于某系统模型有：
$$
\mathbb x_t = g(\mathbb x_{t-1}, \mathbb u_t) + e_t
$$
其中，$e_t$表示过程误差。

对于观测模型有：
$$
\mathbb z_t = h(\mathbb x_t) + \delta_t
$$
其中，$\delta_t$是观测误差。

函数$g(\mathbb x_{t-1}, \mathbb u_t)和h(\mathbb x_t)$可以是非线性函数，所以需要先将这两个函数做泰勒展开，有：
$$
g(\mathbb x_{t-1}, \mathbb u_t) \approx g(\mathbb \mu_{t-1}, \mathbb u_t) + g'(\mathbb \mu_{t-1}, \mathbb u_t) (\mathbb x_{t-1} - \mathbb \mu_{t-1}) \\
= g(\mathbb \mu_{t-1}, \mathbb u_t) + G_t (\mathbb x_{t-1} - \mathbb \mu_{t-1})
$$

$$
h(\mathbb x_t) \approx h(\mathbb \mu_t) + h'(\mathbb \mu_t) (\mathbb x_t - \mathbb \mu_t) \\
= h(\mathbb \mu_t) + H_t (\mathbb x_t - \mathbb \mu_t)
$$

其中，$G_t, H_t$分别表示预测模型和观测模型的雅克比矩阵。

**预测**
$$
\mathbb x_t = g(\mathbb x_{t-1}, \mathbb u_t) \\
P_t = G_tP_tG_t^T + Q_t
$$
**更新**
$$
Residual_t = \mathbb z_t - h(\mathbb x_t) \\
S = H_t P_t H_t^T + R_t \\
K_t = P_t H_t^T S^{-1} \\
\mathbb x_t = \mathbb x_t + K_t Residual_t \\
P_t = P_t - K_t H_t P_t
$$


## 无迹卡尔曼滤波

主要思路：采样数值方法。因为系统比较复杂，直接采用蒙特卡罗方法，随机样本模拟，然后统计样本的均值和方差作为运动后的均值和方差。

关键问题：如何选取样本？

**无迹变换(Unscented Transform)**

核心思想：近似一种概率分布比近似任意一个非线性函数或非线性变换容易。

假设n维变量$\mathbb x$ 服从某均值为$\mathbb \mu$协方差为$P_x$的分布，变量经过某变换$\mathbb f$，得到$\mathbb y$，有$\mathbb y = \mathbb f(\mathbb x)$求$\mathbb y$的均值和分布。

UT变换算法如下：

1. 根据$\mathbb \mu, P$采用一定的采样策略获得$sigma$点集${\chi_i}$。
   $$
   \chi_0 = \mathbb \mu
   $$

   $$
   \chi_i = \mathbb \mu + (\sqrt{(n + \lambda) P_x})_i \quad i = 1 \cdots n
   $$

   $$
   \chi_{i+n} = \mu - (\sqrt{(n + \lambda) P_x})_i \quad i = 1 \cdots n
   $$

   $w_i^m，w_i^c$分别是均值和方差的权重。第一个点的均值权重和方差权重为：
   $$
   w_0^m = \frac {\lambda} { n + \lambda} \\
   w_0^c = \frac {\lambda} {n + \lambda} + 1 - \alpha^2 + \beta
   $$
   剩下点的均值权重和方差权重为：
   $$
   w_i^m = w_i^c = \frac {1} {2(n + \lambda)} \quad i = 1 \cdots 2n
   $$
   $\alpha, \beta, \lambda$是输入的常量，也是需要调整的参数，
   $$
   \alpha \in (0, 1] \\
   \beta = 2  (高斯分布时的最优取值)\\
   \lambda = \alpha^2(n + k) - n \\
   k \ge 0
   $$
   其中$k$影响着采样点到均值点的距离。

2. 对$sigma$中的每个点分别做变换得到点集${\mathbb y_i}$

3. 对点集$\mathbb y_i$，依据权重分别计算均值和方差，$\bar {\mathbb y}, P_y$，
   $$
   \bar {\mathbb y} = \Sigma_{i=0}^{2n} w_i^{m} \mathbb y_i \\
   P_y = \Sigma_{i=0}^{2n} w_i^c (\mathbb y_i - \bar {\mathbb y}) (\mathbb y_i - \bar {\mathbb y})^T
   $$


UKF算法过程

**预测**

1. 根据UT变换生成$sigma$点集$\Chi_{t-1}$
2. 对点集$\Chi_{t-1}$中的每一个点分别执行$g(u_t, \chi_i)$
3. 根据式（13）计算点集均值和方差。

**更新**

1. 根据UT变换生成sigma点集$\Psi_{t}$

2. 对点集$\Psi_t$中的每一个点执行观测变换函数$h(\psi_i)$,得到点集$\mathcal Z_t$

3. 计算点集$\mathcal Z_t$的均值和协方差矩阵
   $$
   \bar {\mathcal z_t} = \Sigma_{i = 0} ^ {2n} w_i^{m} Z_t^i \\
   S_t = \Sigma_{i=0} ^ {2n} w_i^c (Z_t^i - \bar {\mathcal z_t}) (Z_t^i - \bar {\mathcal z_t}) ^ T + Q_t
   $$

4. 计算x到z的变换矩阵
   $$
   P_t^{x, z} = \Sigma_{i=0}^{2n} w_i^c (\Psi_t^i - \mathbb \mu_t) (Z_t^i - \bar {\mathcal z_t})^T
   $$

5. 计算增益$K_t = P_t^{x, z} S_t$

6. 更新均值和方差
   $$
   \mu_t = \mu_t + K_t (z_t - \bar {\mathcal z_t})
   P_t = P_t - K_tS_tK_t^T
   $$
   

## 应用

