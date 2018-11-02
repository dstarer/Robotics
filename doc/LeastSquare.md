## 最小二乘法

------

说明：网上关于最小二乘法的介绍也挺多了，这里只是做一些简单的总结，包括一些常用的求解方式方法，并不赘述其推导过程。

### 最小二乘法与最大似然估计

对于一批数据$(\mathbb x_i ,\mathbb y_i)$，通常会试图寻找一个模型拟合这批数据，也即
$$
\mathbb y_i = \mathbb f(\mathbb x_i)
$$
但是实际过程中数据是存在噪声的，所以模型也表示为：
$$
\mathbb y_i = \mathbb f(\mathbb x_i) + \eta_i
$$
通常假设$\eta_i$是符合$N(0, \Sigma)$的高斯分布，也即
$$
\mathbb y_i - \mathbb f(\mathbb x_i) \sim N(0, \Sigma)
$$
为了求解使得概率最大的$\mathbb y_i - \mathbb f(\mathbb x_i)$，实际也就是使得
$$
(\mathbb y_i -\mathbb f(\mathbb x_i))^T \Sigma^{-1} (\mathbb y_i - \mathbb f(\mathbb x_i))
$$
最小。

$J(\mathbb x_i, \mathbb y_i)$是代价函数，有：
$$
J(\mathbb x_i, \mathbb y_i) =(\mathbb y_i - \mathbb f(\mathbb x_i))^T\Sigma^{-1}(\mathbb y_i - \mathbb f(\mathbb x_i))
$$
如果$\Sigma$是个对角矩阵，也就表示，这里只关心不同变量/状态/特征的权重。如果是$\Sigma$是个一般的协方差矩阵，也就表示，不仅考虑变量/状态/特征的权重，还考虑两者之间的相关性。

特别地，当$\Sigma$是单位矩阵时，
$$
J(\mathbb x_i, \mathbb y_i) = (\mathbb y_i - \mathbb f(\mathbb x_i))^T (\mathbb y_i - \mathbb f(\mathbb x_i))
$$
如果$\mathbb y_i$是实值，则有
$$
J(\mathbb x_i, y_i) = (y_i - f(\mathbb x_i))^2
$$
对$ J(\mathbb x_i, \mathbb y_i)$求导有：
$$
\nabla J(\mathbb x_i, \mathbb y_i) = 2H^T\Sigma^{-1}(\mathbb y_i - \mathbb f(\mathbb x_i))
$$
其中，$H$是$\mathbb y_i - \mathbb f(\mathbb x_i)$的一阶导，即雅克比矩阵。

### 最小二乘法

一个简单的最小二乘法定义如下：
$$
\min_{\mathbb x} \frac{1}{2} \left \|\mathbb f(\mathbb x) \right \|_{2}^2
$$
如果$\mathbb f$是数学形式简单的函数，可直接使用解析形式求解，即求一阶导数，并令一阶导数为0，求得极值，选取其中使得代价函数最小的一组解，作为最优解。

但是通常$\mathbb f$比较复杂，需要使用迭代的方式求解。

对于一个更一般的实值代价函数$F(\mathbb x)$，为了求解最优解，通常做泰勒展开处理：
$$
F(\mathbb x + \Delta \mathbb x) \approx F(\mathbb x) + J(\mathbb x) \Delta \mathbb x +\frac{1}{2} (\Delta \mathbb x)^TH(\mathbb x) \Delta \mathbb x
$$
其中，$J(x)$是$F(\mathbb x)$的雅克比矩阵，$H(\mathbb x)$是海塞矩阵。

对于一般的向量值函数$\mathbb f(\mathbb x)$通常做一阶泰勒展开(二阶导矩阵是个高阶张量，一般不讨论)
$$
\mathbb f(\mathbb x + \Delta \mathbb x) \approx \mathbb f(\mathbb x) +  J(\mathbb x) \Delta \mathbb x
$$
其中，$J(\mathbb x)$是$\mathbb f(\mathbb x)$的雅克比矩阵。

考虑公式(10)，直接求解$\mathbb x$比较复杂，我们考虑求解$\Delta \mathbb x$，然后让$\mathbb x$沿着梯度下降的方向走。

对$\Delta \mathbb x$求导有
$$
\nabla _{\Delta x}F(\mathbb x + \Delta \mathbb x) \approx J^T(\mathbb x) + \frac {1} {2} (H(\mathbb x) + H^T(\mathbb x)) \Delta x
\\= J^T(\mathbb x) + H(\mathbb x) \Delta \mathbb x
$$
令导数为0，则有：
$$
H(\mathbb x) \Delta x = - J^T(\mathbb x)
$$
求解步骤如下：

1. 给定初值$\mathbb x_0$

2. while True do

   ​	根据式（13）计算$\Delta x$

   ​	若$\Delta x$足够小，即$(\Delta \mathbb x)^T\Delta \mathbb x = 0$，则停止

   ​	否则，$\mathbb x = \mathbb x + \Delta \mathbb x$

这种方式，通常由于$\mathbb x$的维度比较大，难以计算$H(\mathbb x)$，所以通常不采用这种方式求解。对于一些只知道$F(\mathbb x)$的函数可以采用这种方式求解，激光点云数据配准的NDT方法实际就是采用这种方法求解的。

### 高斯牛顿法

对于公式（9），可以对$\mathbb f(\mathbb x)$展开，也即式（11），所以有：
$$
F(\mathbb x + \Delta \mathbb x) = \frac{1}{2} \left \| \mathbb f(\mathbb x)\right\|_2^2
\\ 
= \frac{1}{2} \mathbb f^T(\mathbb x) \mathbb f(\mathbb x)
\\
\approx \frac{1}{2} (\mathbb f(\mathbb x) + J(\mathbb x) \Delta \mathbb x)^T (\mathbb f(\mathbb x) + J(\mathbb x) \Delta \mathbb x)
\\
= \frac{1}{2} (\mathbb f^T(\mathbb x) \mathbb f(\mathbb x) + 2 \mathbb f^T(\mathbb x) J(\mathbb x)\Delta \mathbb x + \Delta \mathbb x^T J^T(\mathbb x) J(\mathbb x) \Delta \mathbb x)
$$
对$\Delta \mathbb x$求导有：
$$
\nabla _{\Delta x} F(\mathbb x + \Delta \mathbb x) \approx  J^T(\mathbb x) \mathbb f(\mathbb x)  + J^T(\mathbb x)J(\mathbb x) \Delta \mathbb x
$$
令式（15）为0，
$$
J^T(\mathbb x) J(\mathbb x) \Delta \mathbb x = -  J^T(\mathbb x) \mathbb f(\mathbb x)
$$
式（16）也被称为增量方程，也记做
$$
H \Delta \mathbb x = g
$$
求解步骤如下：

 1. 给定初值$\mathbb x_0$

 2. while True do 

    ​	分别计算$\mathbb f(\mathbb x), J(\mathbb x)$

    ​	根据式（16）计算$\Delta \mathbb x$

    ​	若$\Delta \mathbb x$足够小，则停止，

    ​	否则，$\mathbb x = \mathbb x + \Delta \mathbb x$

由于通常要求$H$是一个正定可逆的矩阵，但是这一点很难得到满足，并且通常为了保证近似的准确性，需要限制$\Delta \mathbb x$的取值。所以通常会给$\Delta \mathbb x$乘上一个系数$\alpha$,即$\mathbb f(\mathbb x + \alpha \Delta \mathbb x)$

### 列文伯格-马夸尔特方法

求解$\Delta \mathbb x$时限定范围，
$$
\mathbb f(\mathbb x + \Delta \mathbb x) - \mathbb f(\mathbb x) = \rho J(\mathbb x) \Delta \mathbb x
$$
其中$\rho$是比例系数，$\rho$接近于1,则近似效果好，如果$\rho$比较小，则说明需要缩小$\Delta \mathbb x$的范围，如果$\rho$比较大，则说明需要扩大$\Delta \mathbb x$的取值范围。

求解算法如下：

 1. 给定初始$\mathbb x_0$

 2. while True do 

    ​	求解优化公式
    $$
    \min_{\Delta \mathbb x} \frac {1}{2} \left \| \mathbb f(\mathbb x_k) + J(\mathbb x_k) \Delta \mathbb x_k \right \|_2^2,  s.t.  \left \| D \Delta \mathbb x_k \right \|_2^2 \leq \mu
    $$
    ​	计算$\rho$

    ​	若$\rho > \frac {3} {4}$，则$\mu = 2 \mu$

    ​	若$\rho < \frac{1}{4}$，则$\mu=0.5\mu$

    ​	如果$\rho$大于阈值，则令$\mathbb x_{k+1} = \mathbb x_k + \Delta \mathbb x$

    ​	判断是否收敛，收敛则跳出。

公式（19）中，方阵D将$\Delta \mathbb x_k$限制在了一个半径为$\mu$的球中，通过拉格朗日算子将式（19）转化为无约束的最优化问题：
$$
\min_{\Delta \mathbb x} \frac {1} {2} \left \| \mathbb f(\mathbb x_k) + J(\mathbb x_k) \Delta \mathbb x_k \right \|_2^2 + \frac {\lambda} {2} \left \| D \Delta \mathbb x \right \|_2^2
$$
求导得：
$$
(J^T(\mathbb x)J(\mathbb x) + \lambda D^TD) \Delta \mathbb x = - J^T(\mathbb x) \mathbb f(\mathbb x)
$$

### 其他求解算法

### 求解库或者求解包

#### ceres

#### g2o



### Reference

1. 《视觉SLAM十四讲》，第6章
2. 