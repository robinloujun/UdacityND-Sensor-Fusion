/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

double uniformRandom(void) { return static_cast<double>(rand()) / RAND_MAX; }

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (uniformRandom() - 0.5);
    double ry = 2 * (uniformRandom() - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (uniformRandom() - 0.5);
    double ry = 2 * (uniformRandom() - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  float a, b, c;

  // For max iterations
  while (maxIterations--) {
    // Randomly sample subset and fit line
    std::unordered_set<int> inliers;
    while (inliers.size() < 2)
      inliers.insert(rand() % cloud->points.size());

    auto iterator = inliers.begin();
    float x1 = cloud->points[*iterator].x;
    float y1 = cloud->points[*iterator].y;
    iterator++;
    float x2 = cloud->points[*iterator].x;
    float y2 = cloud->points[*iterator].y;

    a = y1 - y2;
    b = x2 - x1;
    c = x1 * y2 - x2 * y1;

    for (int i = 0; i < cloud->points.size(); ++i) {
      if (inliers.count(i) > 0)
        continue;

      // Measure distance between every point and fitted line
      float x0 = cloud->points[i].x;
      float y0 = cloud->points[i].y;
      float dist = fabs(a * x0 + b * y0 + c) / sqrt(a * a + b * b);

      // If distance is smaller than threshold count it as inlier
      if (dist < distanceTol)
        inliers.insert(i);
    }

    if (inliers.size() > inliersResult.size())
      inliersResult = inliers;
  }

  // Return indicies of inliers from fitted line with most inliers
  return inliersResult;
}

std::unordered_set<int> planeRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  float a, b, c, d;

  // For max iterations
  while (maxIterations--) {
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

int main() {
  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  std::unordered_set<int> inliers = planeRANSAC(cloud, 10, 0.2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
