# 点云数据处理



### 点云过滤方法

1. **带通滤波器**

   仅保留某个范围内的点，例如，仅保留坐标z取值在0.0, 1.0范围内的点：

   ```C++
   pcl::PassThrough<pcl::PointXYZ> pass;
   pass.setInputCloud(cloud);
   pass.setFilterFieldName("z");
   pass.setFilterLimits(0.0, 1.0);
   pass.filter(*cloud_filtered);
   ```

2. **VoxelGrid滤波器**
   点云下采样中常用的一种方法，通过将三维空间网格化，在每一个voxel(长方体)中，计算点云的重心来表示Voxel中的点云，能比较好的近似原点云数据。

   **Notice** LeafSize的值不要设置的过大，越大点云近似效果就越差，长方体的各边，取值例如，0.1, 0.1, 0.1，过小的时候，经常容易报Too small Leaf Size，这个是因为网格话后voxel中没有点。

   ```C++
   pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
   filter.setInputCloud(cloud);
   filter.setLeafSize(0.01f, 0.01f, 0.01f); // 0.01 * 0.01 * 0.01的立方体
   filter.filter(*cloud_filtered)
   ```

3. **StatisticalOutlierRemoval**离群点过滤

   ```
   算法主体思想：
   	StatisticalOutlierRemoval(k, stddev)
   	dist = []
   	for point p and index in pointcloud:
   		s = search k nearest-neighbors // k-d tree
   		calc the mean dist m_dist from p to all points in s.
   		append m_dist to dist
   	end for
   	calc the mean value u of array dist
   	for i in range(dist):
   		calc the gauss probabilities prob of dist[i] in guass distributions (u, stddev)
   		if prob smaller than 3-sigma probabilities or 4-sigma probabilities
   			remove i from pointcloud.
   ```

   ```C++
   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
   sor.setInputCloud(cloud);
   sor.setMeanK(50); // k
   sor.setStddevMulThresh (1.0); // stddev
   sor.filter(*cloud_filtered);
   ```

4. 点云投影处理

   将点云投影到某个定义的平面或者曲面或者三维空间内，可以用于做点云分割。

   ```c++
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCofficients());
   coefficients->values.resize(4); //example z = 0 plane.
   coefficients->values[0] = 0;
   coefficients->values[1] = 0;
   coefficients->values[2] = 1;
   coefficients->values[3] = 0;
   pcl::ProjectInliers<pcl::PointXYZ> proj;
   proj.setModelType(pcl::SACMODEL_PLANE);
   proj.setInputCloud(cloud);
   proj.setModelCoefficients(cofficients);
   proj.filter(*cloud_filtered);
   ```

5. 径向离群点检测

   基本条件：某个点最近邻点中距离小于d的点不少于k个。

   ```C++
   pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
   // build the filter
   outrem.setInputCloud(cloud);
   outrem.setRadiusSearch(0.8);
   outrem.setMinNeighborsInRadius (2);
   // apply filter
   outrem.filter (*cloud_filtered);
   ```

6. 条件检测

   定义多个计算函数用于组合处理，有或组合或者是与组合，减少循环次数，仅用于基础数值的比较，不可包含复杂的条件。

   ```C++
   pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
   pcl::ConditionAnd<pcl::PointXYZ> ());
   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
   pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
   range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
   pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
   // build the filter
   pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
   condrem.setCondition (range_cond);
   condrem.setInputCloud (cloud);
   condrem.setKeepOrganized(true);
   // apply filter
   condrem.filter (*cloud_filtered);
   ```

7. 

