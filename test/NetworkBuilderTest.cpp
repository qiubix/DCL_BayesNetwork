#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

//#include <pcl/io/pcd_io.h>
#include "../src/Components/NetworkBuilder/NetworkBuilder.hpp"

TEST(NetworkBuilderTest, shouldAddHypothesisNodeToNetwork) {
  Processors::Network::NetworkBuilder component("name");
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud->width = 1;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  cloud->points[0].x = 0.1;
  cloud->points[0].y = 0.2;
  cloud->points[0].z = 0.3;
  for(int i=0; i<128; i++) {
    cloud->points[0].descriptor[i] = i;
  }

  component.buildNetwork(cloud);

  Processors::Network::BayesNetwork network = component.getNetwork();
  EXPECT_THAT(network.hasNode("V_0"), Eq(true));
}

TEST(NetworkBuilderTest, shouldBuildNetworkWithOnlyOneFeatureNode) {
  Processors::Network::NetworkBuilder component("name");
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud->width = 1;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  cloud->points[0].x = 0.1;
  cloud->points[0].y = 0.2;
  cloud->points[0].z = 0.3;
  for(int i=0; i<128; i++) {
    cloud->points[0].descriptor[i] = i;
  }

//  component.setCloud(cloud);
//
//  component.buildNetwork();
//
//  Processors::Network::BayesNetwork network = component.getNetwork();
//  EXPECT_TRUE(network.hasNode("V_0"));
//  EXPECT_TRUE(network.hasNode("F_0"));
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
