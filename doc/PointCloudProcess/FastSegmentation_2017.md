# Fast Segmetation of 3D Point Clouds



### Framework

两步走：

1. 地面检测
2. 点云聚类分割

### Ground Detection

借鉴RANSC思路，区别在于生成候选点集的方式不同。

```python
输入： 点云 P
max_iter：迭代次数

GroudDetection(P):
    P_g = extractInitialSeeds(P)
    for i <- 1 : max_iter :
        model = EstimatePlaneModel(P_g);
        P_g = {}
        P_ng = {}
        for point p in PointCloud P:
            if model(p) < dist_th:
                add p to P_g；
            else
                add p to P_ng;
    return P_g, P_ng

extractInitialSeeds(P):
	P_sorted = SortOnHeightIncrease(P)
	LPR = Average(P_sorted(1: N_LPR));
	P_g = {}
	for i in 1: LNR:
		if P_sorted[i].z < Th_LNR.height + Th_seeds:
			add p_sorted[i] to P_g
	return P_g
```

平面估计算法:
$$
ax + by + cz + d = 0 \\
ax + by + cz = -d  \\
\mathbb n \cdot \mathbb x = -d
$$
其中$\mathbb n$是平面的法向量，而计算$\mathbb n$可以通过对点云数据做PCA分析得到。

具体PCA分析与最小二乘法，以及面的法向量计算，可以参考[PCA推导及点云法向量推导]()

### Segmetation/ Clustering

```python
初始化
P: 输入点云
N_scanlines: 点云线数
Th_run: 同一scanline下(可以理解为水平方向上)，类内间距阈值
Th_merge: 垂直方向上，雷内间距
newLabel: 初始标签

segmentation:
	runsAbove = FindRuns(scanline_1)
	for i = 1: |runsAbove|:
		runsAbove_i.label = new Label;
		newLabel ++;
	for i = 2: N_scanlines:
		runsCurrent = FindRuns(scanline_i)
		UpdateLabels(runsCurrent, runsAbove);
		runsAbove = runsCurrent

UpdateLabels(runsCurrent, runsAbove):
	for i = 1: |runsCurrent| :
		for j = 1: |P_runsCurrent_i|:
			P_nn = FindNearestNeighbor(p_j, runsAbove):
			labelsToMerge <- P_nn.label;
		if isEmpty(labelsToMerge) :
			runsCurrent_i.label = newLabel;
			newLabel ++;
		else:
			l_r = min(labelsToMerge);
			runsCurrent_i.label = l_r
			MergeLabels(labelsToMerge);
```

总体思路：

​	将点云数据按照线数和旋转角度，构成一副图。然后在图上计算Two Pass。

​	首先针对每一条线做聚类，聚类方式通过计算相邻两个点之间的间距来判定；

​	不同线之间的聚类，对相邻两条线之间做聚类，依据两点间的间距；需要注意的一点，垂直方向上聚类时，如何考虑label的合并问题，这里可以参考图像处理中的TwoPass方式。（后续会详细补充）

​	最近点计算：这里原文作者不建议直接使用KD树，而是直接通过相邻两个线束上做扫描快速计算。所以需要自己做索引处理。

​	Label合并时，可以考虑用并查集处理。

### Implementation

*近期会补上*

### Result



### Reference

[Fast Segmentation of 3D Point Clouds: A Paradigm on Lidar Data for Autonomous Vehicle Applications]()

