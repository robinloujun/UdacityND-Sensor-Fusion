// PCL lib Functions for processing point clouds
#include "processPointClouds.h"
#include "kdtree.h"
#include <utility>

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  double x_max, y_max, z_max;
  double x_min, y_min, z_min;
  x_max = y_max = z_max = -500;
  x_min = y_min = z_min = 500;

  // voxel grid point reduction
  // reduce number of point cloud to process faster
  pcl::VoxelGrid<PointT> voxelGridFilter;

  voxelGridFilter.setInputCloud(cloud);
  voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);

  typename pcl::PointCloud<PointT>::Ptr filteredCloud(
      new pcl::PointCloud<PointT>());
  voxelGridFilter.filter(*filteredCloud);

  // region based filtering
  // filter inside box removing any points outside of box area
  typename pcl::PointCloud<PointT>::Ptr roiCloud(new pcl::PointCloud<PointT>());

  pcl::CropBox<PointT> regionOfInterest(true);
  regionOfInterest.setMin(minPoint);
  regionOfInterest.setMax(maxPoint);
  regionOfInterest.setInputCloud(filteredCloud);
  regionOfInterest.filter(*roiCloud);

  std::vector<int> indices;

  // remove roof points from car
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
  roof.setInputCloud(roiCloud);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliersIndices{new pcl::PointIndices};
  for (auto point : indices)
    inliersIndices->indices.push_back(point);

  pcl::ExtractIndices<PointT> extractIndices;
  extractIndices.setInputCloud(roiCloud);
  extractIndices.setIndices(inliersIndices);
  extractIndices.setNegative(true);
  extractIndices.filter(*roiCloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return roiCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacleCloud(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr segmentedCloud(
      new pcl::PointCloud<PointT>());

  for (int index : inliers->indices) {
    segmentedCloud->points.push_back(cloud->points[index]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacleCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstacleCloud, segmentedCloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cout << "Could not estimate a planar model for the given point cloud."
              << std::endl;
  }

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  return segResult;
}

template <typename PointT>
void clusterHelper(int index,
                   const typename pcl::PointCloud<PointT>::Ptr &cloud,
                   std::vector<int> &cluster_indices,
                   std::vector<bool> &processed, KdTree<PointT> *tree,
                   float distanceTol) {
  processed[index] = true;
  cluster_indices.push_back(index);

  std::vector<int> nearest = tree->search(cloud->points[index], distanceTol);

  for (auto idx : nearest) {
    if (!processed[idx]) {
      clusterHelper(idx, cloud, cluster_indices, processed, tree, distanceTol);
    }
  }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                 KdTree<PointT> *tree, float distanceTol, int minSize,
                 int maxSize) {
  // return list of indices for each cluster

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::vector<bool> processed(cloud->points.size(), false);

  for (int i = 0; i < cloud->points.size(); ++i) {
    if (processed[i])
      continue;

    std::vector<int> cluster_indices;
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    clusterHelper(i, cloud, cluster_indices, processed, tree, distanceTol);

    if (cluster_indices.size() >= minSize &&
        cluster_indices.size() <= maxSize) {
      for (auto idx : cluster_indices)
        cluster->points.push_back(cloud->points[idx]);

      clusters.push_back(cluster);
    } else {
      for (auto idx : cluster_indices)
        processed[idx] = false;
    }
  }

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // perform euclidean clustering to group detected obstacles
  KdTree<PointT> *kdTree = new KdTree<PointT>();

  for (int i = 0; i < cloud->points.size(); ++i)
    kdTree->insert(cloud->points[i], i);

  std::vector<pcl::PointIndices> clusterIndices;

  clusters =
      euclideanCluster(cloud, kdTree, clusterTolerance, minSize, maxSize);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  float a, b, c, d;

  // For max iterations
  for (int i = 0; i < maxIterations; ++i) {
    // Randomly sample subset and fit plane
    std::unordered_set<int> inliers;
    while (inliers.size() < 3)
      inliers.insert(rand() % cloud->points.size());

    auto iterator = inliers.begin();
    float x1 = cloud->points[*iterator].x;
    float y1 = cloud->points[*iterator].y;
    float z1 = cloud->points[*iterator].z;
    iterator++;
    float x2 = cloud->points[*iterator].x;
    float y2 = cloud->points[*iterator].y;
    float z2 = cloud->points[*iterator].z;
    iterator++;
    float x3 = cloud->points[*iterator].x;
    float y3 = cloud->points[*iterator].y;
    float z3 = cloud->points[*iterator].z;

    a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    d = -(a * x1 + b * y1 + c * z1);

    for (int i = 0; i < cloud->points.size(); ++i) {
      if (inliers.count(i) > 0)
        continue;

      // Measure distance between every point and fitted plane
      float x0 = cloud->points[i].x;
      float y0 = cloud->points[i].y;
      float z0 = cloud->points[i].z;
      float dist =
          fabs(a * x0 + b * y0 + c * z0 + d) / sqrt(a * a + b * b + c * c);

      // If distance is smaller than threshold count it as inlier
      if (dist < distanceTol)
        inliers.insert(i);
    }
    if (inliers.size() > inliersResult.size())
      inliersResult = inliers;
  }

  // Return indicies of inliers from fitted plane with most inliers
  return inliersResult;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}