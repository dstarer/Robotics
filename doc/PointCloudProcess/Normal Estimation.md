# 平面法线估计

法线估计是点云处理中比较重要的一个步骤。这里主要介绍如何计算法线以及其推导过程。

三维中平面经常可以表达成如下形式：
$$
ax + by + cz + d = 0
$$
用向量的方式表达：
$$
\mathbb w^T \mathbb x = -d
$$
其中，$\mathbb w$是平面的法向量。

### 问题定义

给定平面点集$X={\mathbb x_1, \cdots, \mathbb x_n}$,现在要求解法向量 $\mathbb w $ 。

根据点到平面的距离公式（可以由向量点积计算得到）：
$$
f(\mathbb x_i, \mathbb w) = \frac {\mathbb w^T\mathbb x_i + d} {\| \mathbb w \|_2^2}
$$
为了方便计算，我们假设$\mathbb w$是单位向量，也即是$\mathbb w^T \mathbb w = I$

所以，我们可以将式(3)改写为：
$$
f(\mathbb x_i, \mathbb w, d) = \mathbb w^T\mathbb x_i + d \\
s.t. \mathbb w^T\mathbb w = I
$$
由于点都在一个平面上，所以使得距离最小，即使得$\| f\|$最小，也即是：
$$
min\ F(\mathbb w, d) = \frac{1}{n} \Sigma_{i = 1} ^ n \| f(\mathbb x_i, \mathbb w, d) \| \\
s.t. \mathbb w^T \mathbb w = I
$$
由于直接求解$\| f\|$比较困难，这里使用$f^T f$代替$\| f\|$，所以也即是：
$$
min\ F(\mathbb w, d) = \frac{1}{n} \Sigma_{i=1}^n f^T(\mathbb x_i, \mathbb w, d) f(\mathbb x_i, \mathbb w, d) \\
s.t. \mathbb w^T \mathbb w = I
$$
使用拉格朗日算子去掉约束条件有：
$$
F(\mathbb w, d) = \frac{1}{n} \Sigma_{i=1}^n f^T(\mathbb x_i, \mathbb w, d) f(\mathbb x_i, \mathbb w, d) + \lambda  \mathbb w^T \mathbb w  \\
= \frac{1}{n} \Sigma_{i=1}^n (\mathbb x_i^T \mathbb w \mathbb w^T \mathbb x_i + 2d\mathbb w^T\mathbb x_i + d^2) + \lambda \mathbb w^T \mathbb w \\
= \frac{1}{n} \Sigma_{i=1}^n (\mathbb w^T \mathbb x_i \mathbb x_i^T \mathbb w  + 2d\mathbb w^T\mathbb x_i + d^2) + \lambda \mathbb w^T \mathbb w
$$
求导有:
$$
\frac {\nabla F} {\nabla \mathbb w} = \frac{1}{n} \Sigma_{i=1}^{n} (2 \mathbb x_i \mathbb x_i^T \mathbb w + 2d \mathbb x_i) + 2\lambda \mathbb w \\
= 2(\frac{1}{n}\Sigma_{i=1}^n \mathbb x_i \mathbb x_i^T) \mathbb w + 2(\frac{1}{n}\Sigma_{i=1}^n \mathbb x_i)d + 2\lambda \mathbb w
$$

$$
\frac {\nabla F} {\nabla d} =\frac{1}{n} \Sigma_{i=1}^n 2\mathbb w^T \mathbb x_i + 2d \\
= 2 \mathbb w^T (\frac{1}{n} \Sigma_{i=1}^n \mathbb x_i) + 2d
$$

我们令：
$$
\mathbb u = \frac{1}{n} \Sigma_{i=1}^n \mathbb x_i \\
P = \frac{1}{n} \Sigma_{i=1}^n \mathbb x_i \mathbb x_i^T
$$
令式(9)为0，有：
$$
d = - \mathbb w^T \mathbb u = - \mathbb u^T \mathbb w
$$
令式(8)为0，有：
$$
P\mathbb w - \mathbb u \mathbb u^T \mathbb w + \lambda \mathbb w = 0 \\
(P-\mathbb u \mathbb u^T) \mathbb w = \lambda \mathbb w
$$
式(12)是求矩阵的特征值和特征方程。在计算前均值化，P则是协方差矩阵。

**注意**式(12)实质上就是PCA分解，所以这里得到的$\mathbb w$只是一组平面的一组基。对这组基做平面叉乘，就得到了平面的法向量。

**Important**: 假设有特征值$\sigma_0, \sigma_1, \sigma_2$，我们不妨假设$\sigma_0 <= \sigma_1 <= \sigma_2$，则，$\sigma_0​$对应的向量就是法向量。