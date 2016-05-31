#pragma once
#ifndef BAYESNETWORK_POINTCLOUDGENERATOR_HPP
#define BAYESNETWORK_POINTCLOUDGENERATOR_HPP

#include <pcl/point_cloud.h>

struct PointXYZSIFT;

typedef pcl::PointCloud<PointXYZSIFT>::Ptr PointCloud;

PointCloud getPointCloudWithOnePoint();

PointCloud getPointCloudWithThreePoints();

#endif //BAYESNETWORK_POINTCLOUDGENERATOR_HPP
