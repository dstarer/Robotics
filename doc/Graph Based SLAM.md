### Graph Based SLAM

### Basic Theory



### Applications

假设在一个房间内，有N个路标，每个路标包含位置和方向，一辆车在房间内移动，每个时刻可以探测到部分传感器以及其朝向。现在估计车辆的运动轨迹。

**车辆移动模型**

车辆可以由位置、朝向以及速度四个量表示：
$$
x(t+1) = x(t) + v(t)cos(\theta(t))*dt \\
y(t+1) = y(t) + v(t)sin(\theta(t))*dt \\
\theta(t+1) = \theta(t) + \delta(t) * dt \\
v(t+1) = v(t) + a(t) * dt
$$
其中，$\delta(t), a(t)$是t时刻的控制量。

**观测模型**

路标可以用$(x_l, y_l, \alpha_l)$表示，其中$\alpha_l$表示路标的朝向。

假设车上传感器可以测量出路标相对于当前车辆的位置，以及朝向的偏向角度：
$$
\rho_{l, t} = \sqrt{(x_l - x(t))^2 + (y_l - y(t))^2} \\
\phi_{l, t} = arctan2(y_l - y(t), x_l- x(t)) - \theta(t) \\
\psi_{l, t} = \alpha_l - (\phi(mark_l, car_t) + \theta(t))
$$
则可以根据观测量计算传感器实际位置模型，有：
$$
x_l = x_t + \rho_{l, t} cos(\phi_{t, l} + \theta(t)) \\
y_l = y_t + \rho_{l, t} sin(\phi_{t, l} + \theta(t)) \\
\alpha_l = \psi_{t, l} + \theta(t) + \phi_{t, l}
$$
**构建边**

对任意两个时刻$t_1, t_2$，车辆状态有$\mathbb x_{t1}, \mathbb x_{t2}$，如果两者都有观测到同一路标，则可以构建一条边，根据观测模型有：
$$
x_{t_1} + \rho_{l, t_1} cos(\phi_{l, t_1} + \theta(t_1)) = x_{t_2} + \rho_{l, t_2} cos(\phi_{l, t_2} + \theta(t_2)) \\
y_{t_1} + \rho_{l, t_1} sin(\phi_{l, t_1} + \theta(t_1)) = 
y_{t_2} + \rho_{l, t_2} sin(\phi_{l, t_2} + \theta(t_2)) \\
\psi_{l, t_1} + \theta_{t_1} + \phi_{l, t_1} = 
\psi_{l, t_2} + \theta_{t_2} + \phi_{l, t_2}
$$
假设$t_1<t_2$，则有
$$
\mathbb x_{t_2}  = \mathbb g(\mathbb x_{t_1}, u_{t_1} \cdots u_{t_2-1})
$$
其中，$\mathbb g(\mathbb x_{t_1}, u_{t_1} \cdots u_{t_2-1})$是表示依照运动模型，给定初始状态$\mathbb x_{t_1}$，以及一系列输入$u_{t_1} \cdots u_{t_2-1}$,计算得到的车辆位置。

基于此，边可以用以下6个误差项来表示：
$$
f_{t_1, t_2}^0 = x_{t_2} + \rho_{l, t_2} cos(\phi_{l, t_2} + \theta(t_2)) - (x_{t_1} + \rho_{l, t_1} cos(\phi_{l, t_1} + \theta(t_1))) \\
f_{t_1, t_2}^1 = y_{t_2} + \rho_{l, t_2} sin(\phi_{l, t_2} + \theta(t_2)) -( y_{t_1} + \rho_{l, t_1} sin(\phi_{l, t_1} + \theta(t_1)) ) \\
f_{t_1, t_2}^2 = \psi_{l, t_2} + \theta_{t_2} + \phi_{l, t_2} - (\psi_{l, t_1} + \theta_{t_1} + \phi_{l, t_1}) \\
f_{t_1, t_2}^{3-5} = \mathbb x_{t_2} - \mathbb g(\mathbb x_{t_1},u_{t_1} \cdots u_{t_2-1})
$$
我们将两个状态之间的误差函数记做$\mathbb f(\mathbb x_{t_1}, \mathbb x_{t_2})$，所以代价函数可以记为：
$$
F(\mathbb x_{t_1}, \mathbb x_{t_2}) = \mathbb f^T(\mathbb x_{t_1}, \mathbb x_{t_2}) \mathbb f(\mathbb x_{t_1}, \mathbb x_{t_2})
$$
由于实际测量时噪声的存在，包括约束限制，不同的误差项权重不同，会给不同的项乘上不同的系数，也即是：
$$
F(\mathbb x_{t_1}, \mathbb x_{t_2}) = \mathbb f^T(\mathbb x_{t_1}, \mathbb x_{t_2})\ \Omega \ \mathbb f(\mathbb x_{t_1}, \mathbb x_{t_2})
$$
其中$\Omega$也被称作是信息矩阵，是一个对角矩阵。

有了代价函数之后，边可以考虑求解状态，这是一个典型的最优化函数，通过对$\mathbb f(\mathbb x)$做一阶泰勒展开有：
$$
F(\mathbb x) = (\mathbb f(\mathbb x )+ J_f(\mathbb x) \Delta \mathbb x)^T \Omega (\mathbb f(\mathbb x )+ J_f(\mathbb x) \Delta \mathbb x)
$$
推导可得：
$$
F(\mathbb x) = \mathbb f^T(\mathbb x) \Omega \mathbb f(\mathbb x) + 2\mathbb f^T(\mathbb x) \Omega J_f(\mathbb x) \Delta \mathbb x + \Delta ^T \mathbb x J^T_f(\mathbb x) \Omega J_f(\mathbb x) \Delta \mathbb x
$$
对$\Delta \mathbb x$ 求导有：
$$
J_f^T(\mathbb x) \Omega J_f(\mathbb x) \Delta \mathbb x = - J^T_f(\mathbb x) \Omega f(\mathbb x)
$$
所以计算误差向量的雅克比矩阵，这里由于$\mathbb g(\mathbb x)$ 比较复杂，我们略去不予计算，只考虑前三项。

*这里可以考虑将车辆的运动模型，改成平面变换矩阵的形式去做，而不是用运动模型去处理，这样$\mathbb g(\mathbb x)$就比较简单，比较容易求雅克比矩阵。*

*如果想依靠车辆运动模型来计算，一种可行的方式，是通过数值微分的方式，计算雅克比矩阵。*

