#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "../src/Types/Octree.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

class OctreeTest : public Test {
public:
  typedef pcl::PointCloud<PointXYZSIFT>::Ptr PointCloud;
  PointCloud getPointCloudWithOnePoint() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud->width = 1;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->points[0].x = 0.1;
    cloud->points[0].y = 0.2;
    cloud->points[0].z = 0.3;
    for(int i=0; i<128; i++) cloud->points[0].descriptor[i] = i;
    return cloud;
  }

  PointCloud getPointCloudWithTwoPoints() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud->width = 2;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->points[0].x = 0.1;
    cloud->points[0].y = 0.2;
    cloud->points[0].z = 0.3;
    cloud->points[1].x = 1.1;
    cloud->points[1].y = 1.2;
    cloud->points[1].z = 1.3;
    for(int i=0; i<128; i++) {
      cloud->points[0].descriptor[i] = i;
      cloud->points[1].descriptor[i] = i;
    }
    return cloud;
  }

};

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

  Processors::Network::Octree::DepthFirstIterator it = octree.depthBegin();

  ASSERT_THAT(it->isBranchNode(), Eq(true));
  ASSERT_THAT(it->getCurrentOctreeDepth(), Eq(0));
}

TEST_F(OctreeTest, shouldGetNextOctreeNodeInDepthSearch) {
  PointCloud cloud = getPointCloudWithOnePoint();
  Processors::Network::Octree octree(cloud);
  octree.init();
  Processors::Network::Octree::DepthFirstIterator it = octree.depthBegin();

  ++it;

  EXPECT_THAT(it->isLeafNode(), Eq(true));
  EXPECT_THAT(it->getCurrentOctreeDepth(), Eq(1));
}

TEST_F(OctreeTest, shouldGetLastOctreeNodeInDepthSearch) {
  PointCloud cloud = getPointCloudWithTwoPoints();
  Processors::Network::Octree octree(cloud);
  octree.init();
  Processors::Network::Octree::DepthFirstIterator it = octree.depthBegin();
  Processors::Network::Octree::DepthFirstIterator next = octree.depthBegin();

  const Processors::Network::Octree::DepthFirstIterator end = octree.depthEnd();
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
