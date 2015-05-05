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

  Octree createOctreeWithOnePoint() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud->width = 1;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->points[0].x = 0.1;
    cloud->points[0].y = 0.2;
    cloud->points[0].z = 0.3;
    for(int i=0; i<128; i++) cloud->points[0].descriptor[i] = i;
    Octree octree(128.0f);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    return octree;
  }

  Octree createOctreeWithTwoPoints() {
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
    Octree octree(128.0f);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    return octree;
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

class OctreeLeafNodeTest : public OctreeNodeTest
{
public:
  OctreeLeafNodeTest() {}
  ~OctreeLeafNodeTest() {}
};

TEST_F(OctreeLeafNodeTest, shouldReturnPointIndices) {
  Octree octree = createOctreeWithTwoPoints();
  Octree::DepthFirstIterator it = octree.depth_begin();
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  Processors::Network::OctreeLeafNode node = it.getCurrentOctreeNode();
  std::vector<int> pointIndices;
  pointIndices.push_back(1);

  ASSERT_EQ(pointIndices, node.getPointIndices());

  ++it;
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  node = it.getCurrentOctreeNode();
  pointIndices[0] = 0;
  ASSERT_EQ(pointIndices, node.getPointIndices());
}

TEST_F(OctreeLeafNodeTest, shouldSetId) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  Processors::Network::OctreeLeafNode node = it.getCurrentOctreeNode();

  node.setId(3);

  ASSERT_EQ(3, node.getId());
}

TEST_F(OctreeLeafNodeTest, shouldGetNumberOfChildren) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  Processors::Network::OctreeLeafNode node = it.getCurrentOctreeNode();

  ASSERT_EQ(1, node.getNumberOfChildren());
}

class OctreeBranchNodeTest : public OctreeNodeTest
{
public:
  OctreeBranchNodeTest() {}
  ~OctreeBranchNodeTest() {}
};

TEST_F(OctreeBranchNodeTest, shouldReturnTrueIfNextNodeIsAlsoBranchNode) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();

  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();
  bool isNextBranchNode = false;
  ++it;
  if ((*it)->getNodeType() == pcl::octree::BRANCH_NODE)
    isNextBranchNode = true;

  ASSERT_EQ(isNextBranchNode, node.nextNodeIsAlsoBranchNode());
}

TEST_F(OctreeBranchNodeTest, shouldHaveOnlyOneChild) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();

  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();
  int numberOfChildren = node.getNumberOfChildren();

  ASSERT_EQ(1, numberOfChildren);
  ASSERT_EQ(1, octree.getLeafCount());
}

TEST_F(OctreeBranchNodeTest, shouldHaveMultipleChildren) {
  Octree octree = createOctreeWithTwoPoints();
  Octree::DepthFirstIterator it = octree.depth_begin();

  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();
  int numberOfChildren = node.getNumberOfChildren();

  ASSERT_EQ(2, numberOfChildren);
}

TEST_F(OctreeBranchNodeTest, shouldSetId) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();
  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();

  node.setId(4);

  ASSERT_EQ(4, node.getId());
}

TEST_F(OctreeBranchNodeTest, shouldGetNumberOfChildren) {
  Octree octree = createOctreeWithTwoPoints();
  Octree::DepthFirstIterator it = octree.depth_begin();
  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();

  ASSERT_EQ(2, node.getNumberOfChildren());
}

TEST_F(OctreeNodeTest, shouldSetId) {
  Octree::DepthFirstIterator it = getSampleOctreeFirstNode();
  Processors::Network::OctreeNode* first = new Processors::Network::OctreeBranchNode(it.getCurrentOctreeNode());
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  Processors::Network::OctreeNode* second = new Processors::Network::OctreeLeafNode(it.getCurrentOctreeNode());
  std::vector<Processors::Network::OctreeNode*> nodes;
  nodes.push_back(first);
  nodes.push_back(second);

  for (int i = 0; i < nodes.size(); i++) {
    nodes[i]->setId(i);
  }

  ASSERT_EQ(0, nodes[0]->getId());
  ASSERT_EQ(1, nodes[1]->getId());
}

TEST_F(OctreeNodeTest, shouldGetNumberOfChildren) {
  Octree octree = createOctreeWithTwoPoints();
  Octree::DepthFirstIterator it = octree.depth_begin();
  Processors::Network::OctreeNode* node = new Processors::Network::OctreeBranchNode(it.getCurrentOctreeNode());

  int numberOfChildren = node->getNumberOfChildren();

  ASSERT_EQ(2, numberOfChildren);
}
