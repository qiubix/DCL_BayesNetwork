#pragma once
#ifndef BAYESNETWORK_POINTCLOUDGENERATOR_HPP
#define BAYESNETWORK_POINTCLOUDGENERATOR_HPP

#include <pcl/point_cloud.h>

struct PointXYZSIFT;

typedef pcl::PointCloud<PointXYZSIFT>::Ptr PointCloud;

pcl::PointCloud<PointXYZSIFT>::Ptr getPointCloudWithOnePoint();

pcl::PointCloud<PointXYZSIFT>::Ptr getPointCloudWithThreePoints();

#endif //BAYESNETWORK_POINTCLOUDGENERATOR_HPP
