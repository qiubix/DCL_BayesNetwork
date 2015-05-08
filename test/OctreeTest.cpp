#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "../src/Components/NetworkBuilder/Octree.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

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
  ASSERT_EQ(0, it->getNodeID());
}

//TODO: implement
TEST(OctreeTest, shouldGetNextOctreeNodeInDepthSearch) {
  ASSERT_TRUE(true);
}

//TODO: implement
TEST(OctreeTest, shouldGetLastOctreeNodeInDepthSearch) {
  ASSERT_TRUE(true);
}
