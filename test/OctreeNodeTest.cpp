#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "Types/Octree.hpp"
#include "Types/OctreeContainers.hpp"
#include "Components/NetworkBuilder/OctreeNode.hpp"
#include "Components/NetworkBuilder/OctreeBranchNode.hpp"
#include "Components/NetworkBuilder/OctreeLeafNode.hpp"

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

  ASSERT_THAT(node.getNodePtr()->getNodeType(), Eq(pcl::octree::BRANCH_NODE));
}

TEST_F(OctreeNodeTest, shouldCopyNode) {
  Octree::DepthFirstIterator it = getSampleOctreeFirstNode();
  Processors::Network::OctreeNode node = it.getCurrentOctreeNode();

  Processors::Network::OctreeNode copy = node;
  Processors::Network::OctreeNode secondCopy(node);

  ASSERT_THAT(copy.getNodePtr()->getNodeType(), Eq(pcl::octree::BRANCH_NODE));
  ASSERT_THAT(secondCopy.getNodePtr()->getNodeType(), Eq(pcl::octree::BRANCH_NODE));
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

  ASSERT_THAT(node.getPointIndices(), Eq(pointIndices));

  ++it;
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  node = it.getCurrentOctreeNode();
  pointIndices[0] = 0;
  ASSERT_THAT(node.getPointIndices(), Eq(pointIndices));
}

TEST_F(OctreeLeafNodeTest, shouldSetId) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  Processors::Network::OctreeLeafNode node = it.getCurrentOctreeNode();

  node.setId(3);

  ASSERT_THAT(node.getId(), Eq(3));
}

TEST_F(OctreeLeafNodeTest, shouldGetNumberOfChildren) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();
  while((*it)->getNodeType() != pcl::octree::LEAF_NODE) ++it;
  Processors::Network::OctreeLeafNode node = it.getCurrentOctreeNode();

  ASSERT_THAT(node.getNumberOfChildren(), Eq(1));
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

  ASSERT_THAT(node.nextNodeIsAlsoBranchNode(), Eq(isNextBranchNode));
}

TEST_F(OctreeBranchNodeTest, shouldHaveOnlyOneChild) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();

  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();
  int numberOfChildren = node.getNumberOfChildren();

  ASSERT_THAT(numberOfChildren, Eq(1));
  ASSERT_THAT(octree.getLeafCount(), Eq(1));
}

TEST_F(OctreeBranchNodeTest, shouldHaveMultipleChildren) {
  Octree octree = createOctreeWithTwoPoints();
  Octree::DepthFirstIterator it = octree.depth_begin();

  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();
  int numberOfChildren = node.getNumberOfChildren();

  ASSERT_THAT(numberOfChildren, Eq(2));
}

TEST_F(OctreeBranchNodeTest, shouldSetId) {
  Octree octree = createOctreeWithOnePoint();
  Octree::DepthFirstIterator it = octree.depth_begin();
  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();

  node.setId(4);

  ASSERT_THAT(node.getId(), Eq(4));
}

TEST_F(OctreeBranchNodeTest, shouldGetNumberOfChildren) {
  Octree octree = createOctreeWithTwoPoints();
  Octree::DepthFirstIterator it = octree.depth_begin();
  Processors::Network::OctreeBranchNode node = it.getCurrentOctreeNode();

  ASSERT_THAT(node.getNumberOfChildren(), Eq(2));
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

  ASSERT_THAT(nodes[0]->getId(), Eq(0));
  ASSERT_THAT(nodes[1]->getId(), Eq(1));
}

TEST_F(OctreeNodeTest, shouldGetNumberOfChildren) {
  Octree octree = createOctreeWithTwoPoints();
  Octree::DepthFirstIterator it = octree.depth_begin();
  Processors::Network::OctreeNode* node = new Processors::Network::OctreeBranchNode(it.getCurrentOctreeNode());

  int numberOfChildren = node->getNumberOfChildren();

  ASSERT_THAT(numberOfChildren, Eq(2));
}
