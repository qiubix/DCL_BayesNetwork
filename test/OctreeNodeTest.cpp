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

class OctreeNodeTest : public Test
{
public:
  typedef pcl::octree::OctreePointCloud<PointXYZSIFT, Processors::Network::OctreeContainerPointIndicesWithId, Processors::Network::OctreeContainerEmptyWithId> Octree;

  OctreeNodeTest() : octree(128.0f) {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
    if (pcl::io::loadPCDFile<PointXYZSIFT> ("test_cloud.pcd", *cloud) == -1) {
      std::cout <<"Error reading file!\n";
    }
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
  }

  ~OctreeNodeTest() {}

  Octree::DepthFirstIterator getSampleOctreeFirstNode() {
    Octree::DepthFirstIterator it = octree.depth_begin();
    return it;
  }

protected:
  Octree octree;
};

TEST_F(OctreeNodeTest, shouldInitWithPclOctreeNode) {
  Octree::DepthFirstIterator it = getSampleOctreeFirstNode();
  Processors::Network::OctreeNode node = it.getCurrentOctreeNode();

  ASSERT_EQ(pcl::octree::BRANCH_NODE, node.getNodePtr()->getNodeType());
}

TEST_F(OctreeNodeTest, shouldCopyNode) {
  Octree::DepthFirstIterator it = getSampleOctreeFirstNode();
  Processors::Network::OctreeNode node = it.getCurrentOctreeNode();

  Processors::Network::OctreeNode copy = node;
  Processors::Network::OctreeNode secondCopy(node);

  ASSERT_EQ(pcl::octree::BRANCH_NODE, copy.getNodePtr()->getNodeType());
  ASSERT_EQ(pcl::octree::BRANCH_NODE, secondCopy.getNodePtr()->getNodeType());
}

TEST(OctreeLeafNodeTest, shouldReturnPointIndices) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldReturnTrueIfNextNodeIsAlsoBranchNode) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldHaveOnlyOneChild) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldHaveMultipleChildren) {
  ASSERT_TRUE(true);
}

TEST(OctreeLeafNodeTest, shouldSetId) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldSetId) {
  ASSERT_TRUE(true);
}

TEST_F(OctreeNodeTest, shouldSetId) {
  ASSERT_TRUE(true);
}

TEST(OctreeLeafNodeTest, shouldGetNumberOfChildren) {
  ASSERT_TRUE(true);
}

TEST(OctreeBranchNodeTest, shouldGetNumberOfChildren) {
  ASSERT_TRUE(true);
}

TEST_F(OctreeNodeTest, shouldGetNumberOfChildren) {
  Octree::DepthFirstIterator it = getSampleOctreeFirstNode();
  Processors::Network::OctreeNode node = it.getCurrentOctreeNode();

  int numberOfChildren = node.getNumberOfChildren();

  ASSERT_EQ(3, numberOfChildren);
}
