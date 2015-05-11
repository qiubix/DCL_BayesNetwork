#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

//#include <pcl/io/pcd_io.h>
#include "../src/Components/NetworkBuilder/NetworkBuilder.hpp"

TEST(NetworkBuilderTest, shouldBuildNetworkWithOnlyOneFeatureNode) {
  /*
   * init component with one-point cloud
   * build network
   * should have one feature node and one voxel node
   */
  Processors::Network::NetworkBuilder component("name");
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
    cloud->points[1].descriptor[i] = 128-i;
  }
//  if (pcl::io::loadPCDFile<PointXYZSIFT> ("test_cloud.pcd", *cloud) == -1) {
//    std::cout <<"Error reading file!\n";
//  }

  component.setCloud(cloud);

  component.buildNetwork();
//  ASSERT_TRUE(false);

  Processors::Network::BayesNetwork network = component.getNetwork();
  EXPECT_TRUE(network.hasNode("V_0"));
  EXPECT_TRUE(network.hasNode("F_0"));
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldBuildNetworkWithMultipleFeatureNodes) {
  /*
   * init component with point cloud
   * build network
   * check if number of feature nodes matches number of points in cloud
   * check if has only one child
   */
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldFillCPTsAcordingToNumberOfParents) {
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldSetDefaultProbabilityValuesForFeatureNodes) {
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldHaveOnlyOneChildNode) {
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldNotHaveCycles) {
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldHaveNodesWithUniqueNames) {
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldWriteNetworkToOutputPort) {
  EXPECT_TRUE(true);
}
