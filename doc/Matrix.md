# 矩阵求导常用操作

**网上关于矩阵求导的教程已经比较多了，这里就不一一介绍，只介绍一些个人觉得比较常用的。**



**约定**

1. 标量用小写字母或希腊字母表示，如$t$,$\alpha$ 等
2. 向量用加粗的小写字母表示，例如，向量$\mathbb{x}$, $x_i$表示具体元素，向量默认是列向量，$\mathbf{x}^T$表示航向量。
3. 矩阵用大写字母表示，如$A$等，其元素记做$a_{i, j}$。大写字母加下标表示不同的矩阵，例如$A_1$, $A_2$
4. 实值函数和矩阵值函数用大写字母表示，如$F(\mathbb x)$, 向量值函数用小写加粗字母表示，如$\mathbb f(\mathbb x)$
5. 其他特殊用途，会具体说明。



矩阵求导实际是对矩阵中的每一个元素分别求导。基本求导有以下几种：
1. 矩阵值函数对实值求导
2. 实值函数求导 
3. 向量值函数求导

多次出现求导法则

链式求导法则

常用求导公式



### 矩阵值函数对实值求导

1. 矩阵值函数对实值求导，等于每一个元素分别对实值求导。

2. 若函数$F(x): R \to R^{n \times m}$，$f_{i,j}$表示矩阵i行，j列的元素，则：
   $$
   (\frac {\partial F(x)} {\partial x})_{i. j} = \frac {\partial f_{i, j}} {\partial x}
   $$
   也记做：$\nabla _x F$

### 实值函数求导

**实值函数对矩阵/向量求导**

1. 实值函数对矩阵求导，等于实值函数对矩阵的每一项分别求导，求导结果与自变量同型
2. 若函数$F(\mathbf{x}): R^{n \times  m} \to R$, 则得到一个$n\times m$的矩阵，且有：

$$
(\frac{\partial F(\mathbb(x))}{\partial \mathbb(x)})_{i, j} = \frac{\partial F(\mathbb{x})}{\partial \mathbb{x}_{i,j}}
$$

 

也可以用劈形算子记做：$\nabla _{\mathbb x}F$

若函数$F(\mathbb x): R^n \to R$是一个实值函数，$F(\mathbb x) = a^T \mathbb x$

 1. 对列向量$\mathbb x$求导
    $$
    F(\mathbb x) = \ a_1 \times x_1  + a_2 \times x_2 + a_3 \times x_3
    $$

    $$
    \frac{\partial F(\mathbb x)}{\partial \mathbb x} = \begin{pmatrix} \ a_1 \\ a_2 \\ a_3 \end{pmatrix}
    \\
    \nabla _{\mathbb x}F = a
    $$

 2. 对行向量求导
    $$
    \frac{\partial F(\mathbb x)}{\partial {\mathbb x}^T} = \begin{pmatrix} \ a_1, \ a_2, \ a_3 \end{pmatrix}
    \\
    \nabla _{\mathbb x} F = a^T
    $$
    


### 向量值函数对向量求导

向量值函数对向量值函数的求导，也就是雅克比矩阵。

1. 若$\mathbb f(\mathbb{x}): R^n \to R^m$，则得到一个$m \times n$的矩阵，记做$\nabla x \mathbb f$：
   $$
   (\frac {\partial \mathbb f(\mathbb x)}{\partial \mathbb x})_{i,j} = \frac {\partial f_i} {\partial x_j}
   $$


$$
\nabla _{\mathbb x} \mathbb f^T(\mathbb x) =  (\nabla \mathbb x \mathbb f)^T
$$

### 向量求导法则

**维度相容规则**

求导结果应当满足前述基本条件，即：

1. 实值对矩阵/向量求导，结果与矩阵/向量同型
2. n维列向量对m维列向量求导，结果一定是n*m的矩阵。

当遇到复杂求导时，可以反复用这条规则及矩阵乘法对中间过程做检验。

**链式法则**

若多个向量依赖关系$\mathbb u \rightarrow \mathbb v \rightarrow \mathbb x$, 则：
$$
\frac {\partial \mathbb u} {\partial \mathbb x} = \frac {\partial u} {\partial v} \frac {\partial v} {\partial x}
$$
**变量多次出现法则**

单独计算函数对自变量的每一次出现的求导，再把结果加起来。



### 常用求导公式

$$
\nabla _{\mathbb x} (A \mathbb x) = A
\\
特别地，\nabla _{\mathbb x} \mathbb x = I
$$

向量内积求导
$$
\nabla _\mathbb x  {\mathbb x^T}A{\mathbb x} = \frac {\partial \dot {\mathbb x}^T A \mathbb x} {\partial x} + \frac {\partial {\mathbb x}^T A \dot {\mathbb x}} {\partial x}
\\
 = I A \mathbb x + \frac{\partial ({\mathbb x}^T A \dot {\mathbb x})^T } {\partial x}
 \\
 = A \mathbb x + \frac {\partial \dot{\mathbb x} A^T {\mathbb x}}{\partial x}
 \\
 = A \mathbb x + I A^T \mathbb x
\\
= A \mathbb x + A^T \mathbb x
$$
若$\mathbb h(\mathbb x)$ $ \mathbb g(\mathbb x)$ 是两个m维的列向量，$A$是$m \times m$的方阵，$J_h(\mathbb x)，J_g(\mathbb x)$ 分别是对应的雅克比矩阵，则
$$
\nabla \mathbb h^T(\mathbb x) A \mathbb g(\mathbb x)  = J_h^T(\mathbb x) A \mathbb g(\mathbb x) + J_g^T(\mathbb x) A \mathbb h (\mathbb x)
$$
若$\alpha(\mathbb x)，\mathbb f(\mathbb x)$分别是关于$\mathbb x$的实值函数和向量值函数，则
$$
\nabla _{\mathbb x} ({\alpha ({\mathbb x} )} {\mathbb f( \mathbb x)}) = {\mathbb f (\mathbb x)} \nabla _{\mathbb x^T} {\alpha(\mathbb x)} + {\alpha(\mathbb x)} \nabla _{\mathbb x} {\mathbb f(\mathbb x)}
$$


### 待补充的内容

