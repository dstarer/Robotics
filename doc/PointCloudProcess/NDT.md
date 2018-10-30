# Normal Distribution Transform

NDT是点云匹配中一种重要的方法。不同于ICP等方法，NDT并不需要事先建立landmark之间的对应关系。

对于ICP匹配方法：

1. 寻找两帧数据间的匹配关系，假设有N对匹配关系${(\mathbb x_{target}^i, \mathbb x_{source}^i)}_{i=1}^n$
2. 优化目标$\min_{T} \Sigma_{i=1}^{n}\|  \mathbb x_{target}^i - T \mathbb x_{source}^i\|^2$ 

点云中，寻找到正确的匹配关系并不容易，所以通常只是用这种方法做粗略估计。

因为点云表达了结构信息，而基于结构匹配会更容易得到稳定的结果。NDT就是一种基于结构的方法。

NDT主要思路是根据参考点云构建一个多元高斯分布，如果变换参数能使得两幅点云数据匹配的比较好，那么变换后的点在参考点云中的概率密度会很大。所以NDT中主要优化的是使得概率密度求和最大的变换参数。

NDT算法及推导过程：

*由于三维空间中变换矩阵的求导涉及到李群，李代数，这部分这里不赘述，如若不太容易理解，可以转换到二维平面下考虑。有时为了简便考虑，也可以使用四元数或者是直接使用RPY角近似计算。*

1. 将参考点云划分成若干个指定大小的网格，并计算每个网格的多维正太分布参数(均值和方差)：
   $$
   \mathbb u = \frac{1}{n} \Sigma_i \mathbb x_{target}^i \\
   Q = \frac{1}{n} \Sigma_i (\mathbb x_i - \mathbb u)(\mathbb x_i - \mathbb u)^T
   $$

2. 对于某一个点$p$经过变换T，得到对应点$q$,则$q$的坐标为：
   $$
   \mathbb x_{q} =f(\mathbb x_p, T) = T \mathbb x_p
   $$




3. 根据高斯分布计算$q$的概率密度：
   $$
   s(\mathbb x_q) =\frac{1}{\sqrt{2\pi}^n \|Q\|} \exp(-\frac{(\mathbb x_q - \mathbb u)^T Q^{-1}(\mathbb x_q - \mathbb u) }{2})
   $$
   所以最终的目标函数为：
   $$
   S=  \Sigma_i s(\mathbb x_{q_i})
   $$
   所以优化目标为：
   $$
   \max_T S = \max_T \Sigma_i s(\mathbb x_{q_i})
   $$
   为了求得T，考虑到维度比较小，可以使用牛顿优化算法对式(5)做优化。所以需要计算S关于T的一阶导和二阶导。

4. 我们计算s对q的一阶导数有:

   令$\mathbb g(\mathbb x_q) = \mathbb x_q - \mathbb u$，则
   $$
   J_{\mathbb g} = \frac{\partial \mathbb g} {\partial \mathbb x_q}  = I
   $$

   $$
   s(\mathbb x_q) = c \exp (- \frac{\mathbb g(\mathbb x_q)^TQ^{-1}\mathbb g(\mathbb x_q)}{2})
   $$

   $$
   \begin{split}
   \frac{\partial s(\mathbb x_q)}{\partial \mathbb x_q} &= - s(\mathbb x_q)J^T_{\mathbb g} Q^{-1}\mathbb g(\mathbb x_q) \\
   &= -s(\mathbb x_q) Q^{-1}\mathbb g(\mathbb x_q)
   \end{split} \tag 8
   $$

   $\mathbb x_q$关于T的导数可以参考李代数部分内容，这里就不展开了。所以，有
   $$
   \frac{\partial s}{\partial T} = -s(\mathbb x_q) Q^{-1}\mathbb g(\mathbb x_q) \frac {\partial \mathbb  x_q}{\partial T} \tag 9
   $$


  接着需要计算s对q的二阶导数,**实值函数的二阶导，是一个方阵**：
$$
   \begin{split}
      \frac{\partial (\nabla s(\mathbb x_q))}{\partial \mathbb x_q} &= Q^{-1}\mathbb g(\mathbb x_q) (-s(\mathbb x_q) Q^{-1} \mathbb g(\mathbb x_q))^T + (-s(\mathbb x_q))  \frac{\partial Q^{-1} g(\mathbb x_q)}{\partial \mathbb x_q} \\
      &= -s(\mathbb x_q)Q^{-1}\mathbb g(\mathbb x_q) \mathbb g^T(\mathbb x_q) Q^{-1} - s(\mathbb x_q) \frac{\mathbb \partial Q^{-1} g(\mathbb x_q)}{\partial \mathbb x_q}
      \end{split} \tag {10}
$$
  下面来看$\frac{\partial Q^{-1} g(\mathbb x_q)}{\partial \mathbb x_q}$的求导，**这里是一个向量值函数对向量求导**，根据矩阵相容原理及求导基本法则，我们知道结果一定是一个n*n的矩阵，假设:
$$
   Q^{-1} = \begin{pmatrix} \cdots \mathbb a_i \cdots \end{pmatrix} \tag{11}
$$
  且$Q^{-1}$是一个对称矩阵,
$$
Q^{-1}\mathbb g(\mathbb x_q) = 
      \begin{bmatrix} \cdots \\ \mathbb a_i^T \mathbb g(\mathbb x_q) \\ \cdots \\
      \end{bmatrix} \tag{12}
$$
对$\mathbb a_i^T \mathbb g(\mathbb x_q)$求导，**这是一个实值函数**，有：
$$
   \frac{\partial \mathbb a_i^T \mathbb g(\mathbb x_q)}{\partial \mathbb x_q} = J^T_g \mathbb a_i = \mathbb a_i \tag{13}
$$
  所以有：
$$
\frac{\partial Q^{-1}\mathbb g(\mathbb x_q)}{\partial \mathbb x_q} =
      \begin{bmatrix} \cdots \\ 
      (\frac {\partial \mathbb a_i^T \mathbb g(\mathbb x_q)}{\partial \mathbb x_q})^T \\ \cdots \\
      \end{bmatrix}  \\
      = Q^{-1} \tag{14}
$$
  所以，式（10）有：
$$
   \frac {\partial ^2 s}{\partial \mathbb x_q} = -s(\mathbb x_q)Q^{-1}\mathbb g(\mathbb x_q)\mathbb g^T(\mathbb x_q) Q^{-1} - s(\mathbb x_q)Q^{-1} \tag{15}
$$
  接着按照链式求导，有：
$$
   \frac {\partial ^2 s}{\partial T}  = (-s(\mathbb x_q)Q^{-1}\mathbb g(\mathbb x_q)\mathbb g^T(\mathbb x_q) Q^{-1} - s(\mathbb x_q)Q^{-1}) \frac{\partial \mathbb x_q}{\partial T} \tag{16}
$$
  根据式(9)和式(16)我们就能构造出一阶雅克比矩阵和Hessian矩阵，然后利用梯度下降法求解。

