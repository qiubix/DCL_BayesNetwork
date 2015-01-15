#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../src/Components/CreateNetworkWithSpacialDependencies/Octree.hpp"
#include "../src/Components/CreateNetworkWithSpacialDependencies/OctreeNode.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

using namespace Processors::Network;

class OctreeTest : public Test {
  public:
    OctreeTest(): cloud(new pcl::PointCloud<PointXYZSIFT>) {
      if (pcl::io::loadPCDFile<PointXYZSIFT> ("test_cloud.pcd", *cloud) == -1) {
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