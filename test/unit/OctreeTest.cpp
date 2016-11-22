#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <utils/PointCloudGenerator.hpp>
//#include <pcl/io/pcd_io.h>

#include "Types/Octree.hpp"
#include "Types/DepthFirstIterator.hpp"
#include "Types/PointXYZSIFT.hpp"

class OctreeTest : public Test {
public:
};

TEST_F(OctreeTest, shouldReturnTrueForEmptyPointCloud) {
  pcl::PointCloud<PointXYZSIFT>::Ptr emptyCloud(new pcl::PointCloud<PointXYZSIFT>);
  Processors::Network::Octree octree(emptyCloud);
  ASSERT_THAT(octree.empty(), Eq(true));
}

TEST_F(OctreeTest, shouldInitializeOctreeWithPointCloud) {
  PointCloud cloud = getPointCloudWithOnePoint();
  Processors::Network::Octree octree(cloud);
  octree.init();

  Processors::Network::Octree::OctreeWithSIFT octreeWithSIFT = octree.getOctreeWithSIFT();
  EXPECT_THAT(octreeWithSIFT.getBranchCount(), Eq(1));
  EXPECT_THAT(octreeWithSIFT.getLeafCount(), Eq(1));
  EXPECT_THAT(octreeWithSIFT.getResolution(), Eq(0.01f));
  EXPECT_THAT(octreeWithSIFT.getTreeDepth(), Eq(1));
}

TEST_F(OctreeTest, shouldGetFirstOctreeNode) {
  PointCloud cloud = getPointCloudWithTwoPoints();
  Processors::Network::Octree octree(cloud);
  octree.init();

  Processors::Network::DepthFirstIterator it = octree.depthBegin();

  ASSERT_THAT(it->isBranchNode(), Eq(true));
  ASSERT_THAT(it->getCurrentOctreeDepth(), Eq(0));
}

TEST_F(OctreeTest, shouldGetNextOctreeNodeInDepthSearch) {
  PointCloud cloud = getPointCloudWithOnePoint();
  Processors::Network::Octree octree(cloud);
  octree.init();
  Processors::Network::DepthFirstIterator it = octree.depthBegin();

  ++it;

  EXPECT_THAT(it->isLeafNode(), Eq(true));
  EXPECT_THAT(it->getCurrentOctreeDepth(), Eq(1));
}

TEST_F(OctreeTest, shouldGetLastOctreeNodeInDepthSearch) {
  PointCloud cloud = getPointCloudWithTwoPoints();
  Processors::Network::Octree octree(cloud);
  octree.init();
  Processors::Network::DepthFirstIterator it = octree.depthBegin();
  Processors::Network::DepthFirstIterator next = octree.depthBegin();

  const Processors::Network::DepthFirstIterator end = octree.depthEnd();
  ++next;
  while(next != end) {
    ++next;
    ++it;
  }

  EXPECT_THAT(it->isLeafNode(), Eq(true));
  EXPECT_THAT(it->getCurrentOctreeDepth(), Eq(7));
}

TEST_F(OctreeTest, shouldGetNumberOfPointsInOctree) {
  PointCloud cloud = getPointCloudWithTwoPoints();
  Processors::Network::Octree octree(cloud);
  octree.init();

  ASSERT_THAT(octree.getNumberOfPoints(), Eq(2));
}

TEST_F(OctreeTest, shouldGetSIFTPointById) {
  PointCloud cloud = getPointCloudWithTwoPoints();
  Processors::Network::Octree octree(cloud);
  octree.init();

  PointXYZSIFT referencePoint;
  referencePoint.pointId = 1;
  referencePoint.x = 1.1;
  referencePoint.y = 1.2;
  referencePoint.z = 1.3;
  for(int i=0; i<128; i++) {
    referencePoint.descriptor[i] = i;
  }
  PointXYZSIFT cloudPoint = octree.getPoint(1);
  ASSERT_THAT(cloudPoint.y, Eq(referencePoint.y));
}
