#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "../src/Components/NetworkBuilder/Octree.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

//TODO: remove duplication in tests - create fixture

TEST(OctreeTest, shouldInitializeOctreeWithPointCloud) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud->width = 1;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  cloud->points[0].x = 0.1;
  cloud->points[0].y = 0.2;
  cloud->points[0].z = 0.3;
  for(int i=0; i<128; i++) cloud->points[0].descriptor[i] = i;
  Processors::Network::Octree octree(cloud);
  octree.init();

  Processors::Network::Octree::OctreeWithSIFT octreeWithSIFT = octree.getOctreeWithSIFT();
  EXPECT_EQ(1, octreeWithSIFT.getBranchCount());
  EXPECT_EQ(1, octreeWithSIFT.getLeafCount());
  EXPECT_EQ(0.01f, octreeWithSIFT.getResolution());
  EXPECT_EQ(1, octreeWithSIFT.getTreeDepth());
}

TEST(OctreeTest, shouldGetFirstOctreeNode) {
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
  Processors::Network::Octree octree(cloud);
  octree.init();

  Processors::Network::Octree::DepthFirstIterator it = octree.depthBegin();

  ASSERT_TRUE(it->isBranchNode());
  ASSERT_EQ(0, it->getCurrentOctreeDepth());
}

TEST(OctreeTest, shouldGetNextOctreeNodeInDepthSearch) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud->width = 1;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  cloud->points[0].x = 0.1;
  cloud->points[0].y = 0.2;
  cloud->points[0].z = 0.3;
  for(int i=0; i<128; i++) cloud->points[0].descriptor[i] = i;
  Processors::Network::Octree octree(cloud);
  octree.init();
  Processors::Network::Octree::DepthFirstIterator it = octree.depthBegin();

  ++it;

  EXPECT_TRUE(it->isLeafNode());
  EXPECT_EQ(1, it->getCurrentOctreeDepth());
}

TEST(OctreeTest, shouldGetLastOctreeNodeInDepthSearch) {
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

  EXPECT_TRUE(it->isLeafNode());
  EXPECT_EQ(7, it->getCurrentOctreeDepth());
}
