#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../src/Components/CreateNetworkWithSpacialDependencies/Octree.hpp"
#include "../src/Components/CreateNetworkWithSpacialDependencies/OctreeNode.hpp"
#include "../src/Components/CreateNetworkWithSpacialDependencies/OctreeBranchNode.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

//using namespace Processors::Network;

class OctreeTest : public Test {
  public:
    OctreeTest(): cloud(new pcl::PointCloud<PointXYZSIFT>) {
      if (pcl::io::loadPCDFile<PointXYZSIFT> ("/home/qiubix/DCL/BayesNetwork/test/test_cloud.pcd", *cloud) == -1) {
        std::cout <<"Error reading file!\n";
      }
    }
    ~OctreeTest() {}
  protected:
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
};

TEST_F(OctreeTest, shouldTestNothing) {
  //pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  //if (pcl::io::loadPCDFile<PointXYZSIFT> ("test_cloud.pcd", *cloud) == -1) {
  //  std::cout <<"Error reading file!\n";
  //}
  ASSERT_TRUE(true);
}

//TODO: implement
TEST_F(OctreeTest, shouldInitializeOctreeWithCloud) {
  /*
   * initialize octree
   * check node count
   */
  Processors::Network::Octree octree(cloud);
  octree.init();
  ASSERT_TRUE(true);
}

//TODO: implement
TEST_F(OctreeTest, shouldInitializeIterator) {
  /*
   * initialize octree with cloud
   * initialize iterator
   * set it to begining of the cloud
   */
  Processors::Network::Octree octree(cloud);
  octree.init();
  Processors::Network::Octree::DepthFirstIterator it = octree.depthBegin();
  //pcl::octree::OctreeNode* node = *it;
  Processors::Network::OctreeNode node = *it;
  ASSERT_EQ(node.getNodeType(), Processors::Network::OCTREE_BRANCH_NODE);
  //Processors::Network::OctreeBranchNode& branchNode = static_cast<Processors::Network::OctreeBranchNode&>(node);
  //Processors::Network::OctreeBranchNode branchNode = octree.getBranchNode(node);
  Processors::Network::OctreeBranchNode branchNode(node);
  ASSERT_EQ(branchNode.getNodeType(), Processors::Network::OCTREE_BRANCH_NODE);
  ASSERT_EQ(-1, branchNode.getId());
  branchNode.setId(1);
  ASSERT_EQ(1, branchNode.getId());
}

//TODO: implement
TEST_F(OctreeTest, shouldGetToOctreeLeafNode) {
  /*
   * initialize octree with cloud
   * initialize iterator
   * get to the leaf node
   * get it's id
   */
}

