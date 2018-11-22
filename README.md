# Robotics
自动驾驶相关各类算法原理分析及实现
项目算法参考[PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)及其他项目。
欢迎补充，先埋个坑，后续会把每部分的文档都给补上

### 基础数学知识补充

[矩阵求导常用操作](./doc/Matrix.md)

[最小二乘法](./doc/LeastSquare.md)

[李群及李代数]

### 滤波算法/信息融合

[贝叶斯滤波框架](./doc/BayesFilter.md)

[粒子滤波](./doc/particle-filter.md)

[卡尔曼滤波](./doc/Kalman.md)

[扩展卡尔曼滤波](./doc/Nonlinear-kalman.md)

[unscented kalman filter](./doc/Nonlinear-kalman.md)

### SLAM基础算法

**基于滤波算法的SLAM算法**

[Extended Kalman based SLAM](doc/SLAM_course.pdf)

[EKF-SLAM简介](doc/ekf_slam.md)

[Fast SLAM]

[基于图优化的SLAM算法](./doc/Graph%20Based%20SLAM.md)

### 轨迹规划

[RRT]

### 轨迹跟踪

[PID]

[pure-pursuit](doc/PathTracking/Pure_Pursuit.md)

[stanley](doc/lane%20detection%20and%20following.md)

**车辆运动学模型介绍**

[车辆运动模型](doc/PathTracking/KinematicModel.md)

[MPC]

[LQR]

### 传感器处理

[IMU]

[GPS+IMU融合]

**Lidar**

[地面提取及点云分割](doc/PointCloudProcess/FastSegmentation_2017.md)

[Radar]

[camera]

### 定位及地图构建

[定位中涉及到的最优化问题]

[ICP]

[NDT](doc/PointCloudProcess/NDT.md)

[三维数据中线配准及平面配准方法]

### 开源代码分析

[LOAM]

**Autoware**

[基于NDT的激光雷达SLAM代码分析](doc/autoware/LidarLocalizer.md)

[点云分割与聚类](doc/autoware/lidar_euclidean_cluster_detector.md)

[点云目标跟踪方法讨论]

**Apollo**

