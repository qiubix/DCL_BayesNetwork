#include <gmock/gmock.h>
using ::testing::Eq;
using ::testing::Ne;
using ::testing::Not;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "../src/Components/PointCloudIndexer/PointCloudIndexer.hpp"

#include "Types/PointXYZSIFT.hpp"
#include "utils/PointCloudGenerator.hpp"

using Processors::Network::PointCloudIndexer;

class PointCloudIndexerTest : public Test {
public:
  PointCloudIndexerTest() {
    indexer = new PointCloudIndexer("indexer");
  }

  PointCloudIndexer* indexer;
};

MATCHER(hasEveryPointWithIdDifferentThanDefaultIndex, "") {
  for (int i = 0; i < arg->size(); ++i) {
    if (arg -> points[i].pointId == DEFAULT_INDEX)
      return false;
  }
  return true;
}

MATCHER(hasEveryPointWithDefaultIndex, "") {
  for (int i = 0; i < arg->size(); ++i) {
    if (arg -> points[i].pointId != DEFAULT_INDEX)
      return false;
  }
  return true;
}

MATCHER(hasEveryPointWithUniqueIndex, "") {
  for (int i=0; i < arg -> size() - 1; i++) {
    int firstPointId = arg -> points[i].pointId;

    for (int j = i + 1; j < arg -> size(); ++j) {
      int secondPointId = arg -> points[j].pointId;
      if (firstPointId == secondPointId)
        return false;
    }

  }
  return true;
}

TEST_F(PointCloudIndexerTest, shouldAcceptPointCloudWithSIFT) {
  indexer -> setPointCloud(getPointCloudWithOnePoint());

  unsigned long numberOfPoints = indexer -> getPointCloud()->points.size();
  ASSERT_THAT(numberOfPoints, Eq(1));
  float descriptor = indexer -> getPointCloud() -> points[0].descriptor[127];
  ASSERT_THAT(descriptor, Eq(127.0));
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithSameNumberOfPointsAsModel) {
  PointCloud pointCloudWithThreePoints = getPointCloudWithThreePointsUnindexed();
  indexer -> setPointCloud(pointCloudWithThreePoints);

  indexer -> indexPoints();

  ASSERT_THAT(indexer -> getPointCloud() -> size(), Eq(3));
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithEveryPointIndexed) {
  indexer -> setPointCloud(getPointCloudWithThreePointsUnindexed());
  PointCloud cloudBeforeIndexing = indexer -> getPointCloud();
  ASSERT_THAT(cloudBeforeIndexing, hasEveryPointWithDefaultIndex());

  indexer -> indexPoints();

  PointCloud cloudAfterIndexing = indexer -> getPointCloud();
  ASSERT_THAT(cloudAfterIndexing, hasEveryPointWithIdDifferentThanDefaultIndex());
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithUniquePointIndices) {
  indexer -> setPointCloud(getPointCloudWithThreePointsUnindexed());

  indexer -> indexPoints();

  PointCloud cloudAfterIndexing = indexer -> getPointCloud();
  ASSERT_THAT(cloudAfterIndexing, hasEveryPointWithUniqueIndex());
}

TEST_F(PointCloudIndexerTest, shouldCreateCloudWithCorrectCoordinatesInPoints)
{
  PointCloud pointCloud = getPointCloudWithThreePointsUnindexed();
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
  PointCloud pointCloud = getPointCloudWithThreePointsUnindexed();
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
