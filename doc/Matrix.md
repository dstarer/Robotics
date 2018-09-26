# 矩阵求导常用操作

**网上关于矩阵求导的教程已经比较多了，这里就不一一介绍，只介绍一些个人觉得比较常用的。**



**约定**





矩阵求导实际是对矩阵中的每一个元素分别求导。常用求导有以下几种：
1. 实值函数求导 
2. 向量值函数求导
3. 链式求导

### 实值函数求导
实值对向量求导，等于实值函数对向量的每一项分别求导，求导结果同自变量类型

例如：

 1. 对列向量求导
    $$
    F(X) = \ a_1 \times x_1  + a_2 \times x_2 + a_3 \times x_3
    $$

    $$
    \frac{\partial F(X)}{\partial X} = \begin{matrix} \ a_1 \\ a_2 \\ a_3 \end{matrix}
    $$

 2. 对行向量求导
    $$
    \frac{\partial F(X)}{\partial X} = \begin{vmatrix} \ a_1, \ a_2, \ a_3 \end{vmatrix}
    $$

 3. 对矩阵求导
    $$
    F(X) = \ a_{11} \times x_{11}  + \ a_{12} \times x_{12} + \ a_{13} \times x_{13} + \ a_{21} \times x_{21} + \ a_{22} \times x_{22} + \ a_{23} \times x_{23}
    $$

    $$
    \frac{\partial F(X)}{\partial X} = \begin{vmatrix} \ a_{11}, \ a_{12}, \ a_{13} \\ a_{21}, \ a_{22}, \ a_{23} \end{vmatrix}
    $$

	4. 几个常用公式

    假设 *X*是n维列向量，*F(X)*是实值函数，
    $$
    F(X) = a^TX
    $$

    $$
    \frac{\partial F(X)}{\partial X} = a^T
    $$

    $$
    F(X) = X^Ta
    $$

    $$
    \frac {\partial F(X)} {\partial X} = a
    $$


### 向量函数求导






