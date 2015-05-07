#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../src/Components/NetworkBuilder/Octree.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

//using namespace Processors::Network;

//TODO: implement
TEST(OctreeTest, shouldInitializeOctreeWithPointCloud) {
  /*
   * initialize octree
   * check node count
   */
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  if (pcl::io::loadPCDFile<PointXYZSIFT> ("test_cloud.pcd", *cloud) == -1) {
    std::cout <<"Error reading file!\n";
  }
  Processors::Network::Octree octree(cloud);
  octree.init();
  ASSERT_TRUE(true);
}

//TODO: implement
TEST(OctreeTest, shouldGetFirstOctreeNode) {
  /*
   * initialize octree with cloud
   * initialize iterator
   * set it to begining of the cloud
   */
  ASSERT_TRUE(true);
}

//TODO: implement
TEST(OctreeTest, shouldGetNextOctreeNodeInDepthSearch) {
  ASSERT_TRUE(true);
}

//TODO: implement
TEST(OctreeTest, shouldGetLastOctreeNodeInDepthSearch) {
  ASSERT_TRUE(true);
}
