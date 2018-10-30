# Euclidean Cluster Detector

三维点云目标检测和分割比较直接，由于点云信息本身带了深度信息，所以很明显一个目标对应的点云是连续并且集中的，表达出来也就是类内间距比较小，而类间间距比较大，所以基于这个先验信息，我们可以使用kmeans聚类算法对点云做聚类，聚类后的结果就是目标。

下面来看autoware中Euclidean Cluster Detector的实现

通常在点云聚类前，我们需要先对点云做预处理，滤除噪声数据。

Euclidean Cluster Detector中使用了以下几种数据的预处理方法：

1. 去除检测中的噪声点，比如距离雷达太近的点和超出lidar量程的点。

   ```C++
   void removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance);
   ```

2. 下采样，用于减少数据量。

   ```C++
   void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2);
   ```

3. 点云裁剪

   ```c++
   void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height = -1.3, float in_max_height = 0.5)；
   ```

4. 仅保留车道上的目标

   ```c++
   void keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold = 1.5,
                       float in_right_lane_threshold = 1.5)
   ```

5. 去除地面

6. ```c++
   void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height = 0.2,
                    float in_floor_max_angle = 0.1)
   ```

   频域滤波，去除频域变化不明显的点（DoN算子）

   ```c++
   void differenceNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);
   ```

聚类：

​	这里聚类主要做了以下几件事情，

 	1. 先转换到将点云投影到x-y平面上，然后在x-y平面上做聚类，得到初始的目标集合；
 	2. 对检测得到的目标集合进行合并优化。

```c++
std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                        jsk_recognition_msgs::BoundingBoxArray& in_out_boundingbox_array,
                                        autoware_msgs::centroids& in_out_centroids,
                                        double in_max_cluster_distance = 0.5)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
  // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++)
  {
    cloud_2d->points[i].z = 0;
  }

  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> cluster_indices;

  // perform clustering on 2d cloud
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(in_max_cluster_distance);  //
  ec.setMinClusterSize(_cluster_size_min);
  ec.setMaxClusterSize(_cluster_size_max);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);
  // use indices on 3d cloud

  /*pcl::ConditionalEuclideanClustering<pcl::PointXYZ> cec (true);
  cec.setInputCloud (in_cloud_ptr);
  cec.setConditionFunction (&independentDistance);
  cec.setMinClusterSize (cluster_size_min);
  cec.setMaxClusterSize (cluster_size_max);
  cec.setClusterTolerance (_distance*2.0f);
  cec.segment (cluster_indices);*/

  /////////////////////////////////
  //---	3. Color clustered points
  /////////////////////////////////
  unsigned int k = 0;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<ClusterPtr> clusters;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color
  // cluster
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int)_colors[k].val[0], (int)_colors[k].val[1],
                      (int)_colors[k].val[2], "", _pose_estimation);
    clusters.push_back(cluster);

    k++;
  }
  // std::cout << "Clusters: " << k << std::endl;
  return clusters;
}
```

目标集合的合并

​	主要依据，判断两个点云的质心距离是否很相近，如果相近，则表明是同一个物体，需要合并。实际就是在图上寻找连通块的过程。

​	实现伪代码：​		

```
initial clusters,
visited = [False] * n
for i: 1, n do
	clusters_should_merged = []
	if !visited[i] :
		findClustersShouldMerge(i, clusters, visited, clusters_should_merged)
end

findClustersShouldMerge(idx, clusters, visited, clusters_should_merged) do
	for i: 1, n do
		if idx != i and !visited[i] do
			if distance(clusters[idx], clusters[i]) <= THRESH:
				visited[i] = true
				push clusters[i] to clusters_should_merged
				findClustersShouldMerge(i, clusters, visited, clusters_should_merged)
end
```



```c++
void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr>& in_clusters,
                       std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
                       double in_merge_threshold)
{
  // std::cout << "checkClusterMerge" << std::endl;
  pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (i != in_cluster_id && !in_out_visited_clusters[i])
    {
      pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
      double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
      if (distance <= in_merge_threshold)
      {
        in_out_visited_clusters[i] = true;
        out_merge_indices.push_back(i);
        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
        checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
      }
    }
  }
}

void mergeClusters(const std::vector<ClusterPtr>& in_clusters, std::vector<ClusterPtr>& out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t& current_index,
                   std::vector<bool>& in_out_merged_clusters)
{
  // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
  pcl::PointCloud<pcl::PointXYZ> mono_cloud;
  ClusterPtr merged_cluster(new Cluster());
  for (size_t i = 0; i < in_merge_indices.size(); i++)
  {
    sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
    in_out_merged_clusters[in_merge_indices[i]] = true;
  }
  std::vector<int> indices(sum_cloud.points.size(), 0);
  for (size_t i = 0; i < sum_cloud.points.size(); i++)
  {
    indices[i] = i;
  }

  if (sum_cloud.points.size() > 0)
  {
    pcl::copyPointCloud(sum_cloud, mono_cloud);
    // std::cout << "mergedClusters " << sum_cloud.points.size() << " mono:" << mono_cloud.points.size() << std::endl;
    // cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int)_colors[k].val[0], (int)_colors[k].val[1],
    // (int)_colors[k].val[2], "", _pose_estimation);
    merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
                             (int)_colors[current_index].val[0], (int)_colors[current_index].val[1],
                             (int)_colors[current_index].val[2], "", _pose_estimation);
    out_clusters.push_back(merged_cluster);
  }
}
```

**Todo： 基于GridMap寻找运动目标**