#include "PointCloudGenerator.hpp"
#include <Types/PointXYZSIFT.hpp>

PointCloud getPointCloudWithOnePoint() {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud -> width = 1;
  cloud -> height = 1;
  cloud -> points.resize(cloud -> width * cloud -> height);
  cloud -> points[0].x = 0.1;
  cloud -> points[0].y = 0.2;
  cloud -> points[0].z = 0.3;
  for(int i=0; i<128; i++) {
    cloud -> points[0].descriptor[i] = i;
  }
  return cloud;
}

PointCloud getPointCloudWithTwoPoints() {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud->width = 2;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  cloud->points[0].pointId = 0;
  cloud->points[0].x = 0.1;
  cloud->points[0].y = 0.2;
  cloud->points[0].z = 0.3;
  cloud->points[1].pointId = 1;
  cloud->points[1].x = 1.1;
  cloud->points[1].y = 1.2;
  cloud->points[1].z = 1.3;
  for(int i=0; i<128; i++) {
    cloud->points[0].descriptor[i] = i;
    cloud->points[1].descriptor[i] = i;
  }
  return cloud;
}

PointCloud getPointCloudWithThreePoints() {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud -> width = 3;
  cloud -> height = 1;
  cloud -> points.resize(cloud -> width * cloud -> height);
  cloud -> points[0].x = 0.0;
  cloud -> points[0].y = 0.0;
  cloud -> points[0].z = 0.0;
  cloud -> points[0].pointId = 0;
  cloud -> points[1].x = 1.0;
  cloud -> points[1].y = 1.0;
  cloud -> points[1].z = 1.0;
  cloud -> points[1].pointId = 2;
  cloud -> points[2].x = 0.0;
  cloud -> points[2].y = 0.0;
  cloud -> points[2].z = 1.0;
  cloud -> points[2].pointId = 4;
  for(int i=0; i<128; i++) {
    cloud -> points[0].descriptor[i] = i;
    cloud -> points[1].descriptor[i] = 128-i;
    cloud -> points[2].descriptor[i] = i*i;
  }
  return cloud;
}
