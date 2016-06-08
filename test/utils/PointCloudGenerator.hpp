#pragma once
#ifndef BAYESNETWORK_POINTCLOUDGENERATOR_HPP
#define BAYESNETWORK_POINTCLOUDGENERATOR_HPP

#include <pcl/point_cloud.h>

struct PointXYZSIFT;

typedef pcl::PointCloud<PointXYZSIFT>::Ptr PointCloud;

static const int DEFAULT_INDEX = 999;

PointCloud getPointCloudWithOnePoint();

PointCloud getPointCloudWithTwoPoints();

PointCloud getPointCloudWithThreePoints();

PointCloud getPointCloudWithThreePointsUnindexed();

#endif //BAYESNETWORK_POINTCLOUDGENERATOR_HPP
