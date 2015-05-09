#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/NetworkBuilder.hpp"

TEST(NetworkBuilderTest, shouldCreateComponentForTesting)
{
  Processors::Network::NetworkBuilder component("name");
  component.prepareInterface();
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldReadSIFTPointCloudFromInputDataPort) {
  EXPECT_TRUE(true);
}

TEST(NetworkBuilderTest, shouldBuildNetworkWithOnlyOneFeatureNode) {
  /*
   * init component with one-point cloud
   * build network
   * should have one feature node and one voxel node
   */
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
