#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "../src/Components/PointCloudIndexer/PointCloudIndexer.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

using Processors::Network::PointCloudIndexer;

class PointCloudIndexerTest : public Test {
public:
  typedef pcl::PointCloud<PointXYZSIFT>::Ptr PointCloud;

  PointCloud getPointCloudWithOnePoint() {
    PointCloud cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud->width = 1;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->points[0].x = 0.1;
    cloud->points[0].y = 0.2;
    cloud->points[0].z = 0.3;
    for(int i=0; i<128; i++) cloud->points[0].descriptor[i] = i;
    return cloud;
  }

  PointCloud getPointCloudWithThreePoints() {
    PointCloud cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud -> width = 3;
    cloud -> height = 1;
    cloud -> points.resize(cloud -> width * cloud -> height);
    cloud -> points[0].x = 0.0;
    cloud -> points[0].y = 0.0;
    cloud -> points[0].z = 0.0;
    cloud -> points[0].pointId = 999;
    cloud -> points[1].x = 1.0;
    cloud -> points[1].y = 1.0;
    cloud -> points[1].z = 1.0;
    cloud -> points[1].pointId = 999;
    cloud -> points[2].x = 0.0;
    cloud -> points[2].y = 0.0;
    cloud -> points[2].z = 1.0;
    cloud -> points[2].pointId = 999;
    for(int i=0; i<128; i++) {
      cloud -> points[0].descriptor[i] = i;
      cloud -> points[1].descriptor[i] = 128-i;
      cloud -> points[2].descriptor[i] = i*i;
    }
    return cloud;
  }
};

TEST_F(PointCloudIndexerTest, shouldAcceptPointCloudWithSIFT) {
  PointCloudIndexer indexer("indexer");

  indexer.setPointCloud(getPointCloudWithOnePoint());
  unsigned long numberOfPoints = indexer.getPointCloud()->points.size();
  ASSERT_THAT(numberOfPoints, Eq(1));
  int descriptor = indexer.getPointCloud() -> points[0].descriptor[127];
  ASSERT_THAT(descriptor, Eq(127));
}

TEST_F(PointCloudIndexerTest, shouldIndexPointsIncrementally) {
  PointCloudIndexer indexer("indexer");
  indexer.setPointCloud(getPointCloudWithThreePoints());
  pcl::PointCloud<PointXYZSIFT>::Ptr cloudBeforeIndexing = indexer.getPointCloud();
  ASSERT_THAT(cloudBeforeIndexing -> points[0].pointId, Eq(999));
  ASSERT_THAT(cloudBeforeIndexing -> points[1].pointId, Eq(999));
  ASSERT_THAT(cloudBeforeIndexing -> points[2].pointId, Eq(999));
  
  indexer.indexPoints();
  
  pcl::PointCloud<PointXYZSIFT>::Ptr cloudAfterIndexing = indexer.getPointCloud();
  ASSERT_THAT(cloudAfterIndexing -> points[0].pointId, Eq(0));
  ASSERT_THAT(cloudAfterIndexing -> points[1].pointId, Eq(1));
  ASSERT_THAT(cloudAfterIndexing -> points[2].pointId, Eq(2));
}
