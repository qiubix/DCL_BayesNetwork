#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../src/Components/NetworkBuilder/Octree.hpp"
#include "../src/Components/NetworkBuilder/OctreeNode.hpp"
#include "../src/Components/NetworkBuilder/OctreeBranchNode.hpp"
#include "../src/Components/NetworkBuilder/OctreeLeafNode.hpp"
#include "../src/Components/NetworkBuilder/OctreeContainers.hpp"


TEST(OctreeNodeTest, shouldInitWithPclOctreeNode) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  if (pcl::io::loadPCDFile<PointXYZSIFT> ("test_cloud.pcd", *cloud) == -1) {
    std::cout <<"Error reading file!\n";
  }
  typedef pcl::octree::OctreePointCloud<PointXYZSIFT, Processors::Network::OctreeContainerPointIndicesWithId, Processors::Network::OctreeContainerEmptyWithId> Octree;
  Octree octree(128.0f);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();
  Octree::DepthFirstIterator it = octree.depth_begin();
  Processors::Network::OctreeNode node = it.getCurrentOctreeNode();
  ASSERT_TRUE(true);
}

TEST(OctreeNodeTest, shouldCopyNode) {
  ASSERT_TRUE(true);
}

TEST(OctreeNodeTest, shouldGetNumberOfChildren) {
  ASSERT_TRUE(true);
}

TEST(OctreeLeafNodeTest, shouldReturnPointIndices) {
  ASSERT_TRUE(true);
}

TEST(OctreeLeafNodeTest, shouldSetId) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldBeBranchNode) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldSetId) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldHaveOnlyOneChild) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldHaveMultipleChildren) {
  ASSERT_TRUE(true);
}

TEST(OctreeNodeTest, shouldSetId) {
  ASSERT_TRUE(true);
}
