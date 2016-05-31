#include <gmock/gmock.h>
using ::testing::Eq;
using ::testing::Ne;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "../src/Components/PointCloudIndexer/PointCloudIndexer.hpp"

#include "Types/PointXYZSIFT.hpp"

using Processors::Network::PointCloudIndexer;

class PointCloudIndexerTest : public Test {
public:
  PointCloudIndexerTest() {
    indexer = new PointCloudIndexer("indexer");
  }

  static const int DEFAULT_INDEX = 999;

  typedef pcl::PointCloud<PointXYZSIFT>::Ptr PointCloud;

  //TODO: extract those methods to separate PointCloudGenerator class
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
    cloud -> points[0].pointId = DEFAULT_INDEX;
    cloud -> points[1].x = 1.0;
    cloud -> points[1].y = 1.0;
    cloud -> points[1].z = 1.0;
    cloud -> points[1].pointId = DEFAULT_INDEX;
    cloud -> points[2].x = 0.0;
    cloud -> points[2].y = 0.0;
    cloud -> points[2].z = 1.0;
    cloud -> points[2].pointId = DEFAULT_INDEX;
    for(int i=0; i<128; i++) {
      cloud -> points[0].descriptor[i] = i;
      cloud -> points[1].descriptor[i] = 128-i;
      cloud -> points[2].descriptor[i] = i*i;
    }
    return cloud;
  }

  PointCloudIndexer* indexer;
};

TEST_F(PointCloudIndexerTest, shouldAcceptPointCloudWithSIFT) {
  indexer -> setPointCloud(getPointCloudWithOnePoint());

  unsigned long numberOfPoints = indexer -> getPointCloud()->points.size();
  ASSERT_THAT(numberOfPoints, Eq(1));
  float descriptor = indexer -> getPointCloud() -> points[0].descriptor[127];
  ASSERT_THAT(descriptor, Eq(127.0));
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithSameNumberOfPointsAsModel) {
  PointCloud pointCloudWithThreePoints = getPointCloudWithThreePoints();
  indexer -> setPointCloud(pointCloudWithThreePoints);

  indexer -> indexPoints();

  ASSERT_THAT(indexer -> getPointCloud() -> size(), Eq(3));
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithEveryPointIndexed) {
  indexer -> setPointCloud(getPointCloudWithThreePoints());
  PointCloud cloudBeforeIndexing = indexer -> getPointCloud();
  for (int i = 0; i < cloudBeforeIndexing->size(); ++i) {
    ASSERT_THAT(cloudBeforeIndexing -> points[i].pointId, Eq(DEFAULT_INDEX));
  }

  indexer -> indexPoints();
  
  PointCloud cloudAfterIndexing = indexer -> getPointCloud();
  for (int j = 0; j < cloudAfterIndexing->size(); ++j) {
    ASSERT_THAT(cloudAfterIndexing -> points[j].pointId, Ne(DEFAULT_INDEX));
  }
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithUniquePointIndices) {
  indexer -> setPointCloud(getPointCloudWithThreePoints());

  indexer -> indexPoints();

  PointCloud cloudAfterIndexing = indexer -> getPointCloud();
  for (int i=0; i < cloudAfterIndexing -> size() - 1; i++) {
    int firstPointId = cloudAfterIndexing -> points[i].pointId;
    for (int j = i + 1; j < cloudAfterIndexing->size(); ++j) {
      int secondPointId = cloudAfterIndexing -> points[j].pointId;
      ASSERT_THAT(firstPointId, Ne(secondPointId));
    }
  }
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithCorrectCoordinatesInPoints)
{
  PointCloud pointCloud = getPointCloudWithThreePoints();
  indexer -> setPointCloud(pointCloud);
  const int POINT_INDEX = 2;
  PointXYZSIFT pointXYZSIFT = pointCloud -> points[POINT_INDEX];

  indexer -> indexPoints();

  PointCloud indexedPointCloud = indexer -> getPointCloud();
  ASSERT_THAT(indexedPointCloud -> points[POINT_INDEX].x, Eq(pointXYZSIFT.x));
  ASSERT_THAT(indexedPointCloud -> points[POINT_INDEX].y, Eq(pointXYZSIFT.y));
  ASSERT_THAT(indexedPointCloud -> points[POINT_INDEX].z, Eq(pointXYZSIFT.z));
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithCorrectSIFTDescriptorInPoints) {
  PointCloud pointCloud = getPointCloudWithThreePoints();
  indexer -> setPointCloud(pointCloud);
  const int POINT_INDEX = 2;
  PointXYZSIFT pointXYZSIFT = pointCloud -> points[POINT_INDEX];

  indexer -> indexPoints();

  PointCloud indexedPointCloud = indexer -> getPointCloud();
  for (int i = 0; i < indexedPointCloud -> size(); ++i) {
    float newSIFTDescriptor = indexedPointCloud -> points[POINT_INDEX].descriptor[i];
    ASSERT_THAT(newSIFTDescriptor, Eq(pointXYZSIFT.descriptor[i]));
  }
}

TEST_F(PointCloudIndexerTest, shouldInitializeComponentHandlers) {
  indexer -> prepareInterface();
  
  std::string handlers = indexer -> listHandlers();
  ASSERT_THAT(handlers, Eq("onNewCloud\n"));
}
