#include <gmock/gmock.h>
using ::testing::Eq;
using ::testing::Le;
#include <gtest/gtest.h>
using ::testing::Test;

//#include <pcl/io/pcd_io.h>
#include <SMILE/network.h>
#include "Types/BayesNetwork.hpp"
#include "Types/CPTManager.hpp"
#include <Types/Octree.hpp>
#include "Components/NetworkBuilder/NetworkBuilder.hpp"
#include "Components/NetworkBuilder/NetworkBuilderExceptions.hpp"

#include "utils/PointCloudGenerator.hpp"

using Processors::Network::Octree;

class NetworkBuilderIntegrationTest : public Test {
public:
  NetworkBuilderIntegrationTest() {
    networkBuilder = new Processors::Network::NetworkBuilder("name");
  }

  Processors::Network::NetworkBuilder* networkBuilder;

};

TEST_F(NetworkBuilderIntegrationTest, shouldThrowExceptionWhenBuildingFromEmptyCloud) {
  pcl::PointCloud<PointXYZSIFT>::Ptr emptyCloud(new pcl::PointCloud<PointXYZSIFT>);
  Octree octree(emptyCloud);

  EXPECT_THROW(networkBuilder -> buildNetwork(&octree), PointCloudIsEmptyException);
}

TEST_F(NetworkBuilderIntegrationTest, shouldAddHypothesisNodeToNetwork) {
  Octree* octreeWithOnePoint = new Octree(getPointCloudWithOnePoint());
  octreeWithOnePoint -> init();
  networkBuilder -> buildNetwork(octreeWithOnePoint);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.hasNode("V_0"), Eq(true));
}

TEST_F(NetworkBuilderIntegrationTest, shouldBuildNetworkWithOnlyOneFeatureNode) {
  Octree* octreeWithOnePoint = new Octree(getPointCloudWithOnePoint());
  octreeWithOnePoint -> init();
  networkBuilder -> buildNetwork(octreeWithOnePoint);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.hasNode("V_0"), Eq(true));
  ASSERT_THAT(network.hasNode("V_1"), Eq(true));
  ASSERT_THAT(network.hasNode("F_0"), Eq(true));
  ASSERT_THAT(network.getNumberOfNodes(), Eq(3));
}

TEST_F(NetworkBuilderIntegrationTest, shouldBuildNetworkWithMultipleFeatureNodes) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();
  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNumberOfNodes(), Eq(7));
  ASSERT_THAT(network.hasNode("F_0"), Eq(true));
  ASSERT_THAT(network.hasNode("F_2"), Eq(true));
  ASSERT_THAT(network.hasNode("F_4"), Eq(true));
}

TEST_F(NetworkBuilderIntegrationTest, shouldHaveTheSameNumberOfFeatureNodesAsPointsInOctree) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();
  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNumberOfFeatureNodes(), Eq(3));
}

TEST_F(NetworkBuilderIntegrationTest, shouldSetDefaultProbabilityValuesForFeatureNodes) {
  Octree* octreeWithOnePoint = new Octree(getPointCloudWithOnePoint());
  octreeWithOnePoint -> init();
  networkBuilder -> buildNetwork(octreeWithOnePoint);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  Processors::Network::CPTManager manager = network.getNextRootNode().getNodeCPTManager();
  //TODO: in C++11 it'll be much simpler
  const double probs[] = { 0.5,0.5 };
  std::vector<double> probabilities(probs, probs+sizeof(probs)/sizeof(double));
  ASSERT_THAT(manager.displayCPT(), Eq(probabilities));
}

TEST_F(NetworkBuilderIntegrationTest, shouldHaveOnlyOneChildNode) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();
  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  Processors::Network::BayesNetworkNode firstNode = network.getNode("V_1");
  Processors::Network::BayesNetworkNode secondNode = network.getNode("V_2");
  Processors::Network::BayesNetworkNode childNode = network.getNode("V_0");

  ASSERT_THAT(firstNode.getChild().getName(), Eq("V_0"));
  ASSERT_THAT(secondNode.getChild().getName(), Eq("V_0"));
  ASSERT_THAT(firstNode.getNumberOfChildren(), Eq(1));
  ASSERT_THAT(secondNode.getNumberOfChildren(), Eq(1));
  ASSERT_THAT(childNode.getNumberOfChildren(), Eq(0));
}

TEST_F(NetworkBuilderIntegrationTest, shouldNotHaveCycles) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();
  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNetwork().IsAcyclic(), Eq(1));
}

TEST_F(NetworkBuilderIntegrationTest, shouldFillCPTsAcordingToNumberOfParents) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderIntegrationTest, shouldHaveNodesWithUniqueNames) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderIntegrationTest, shouldReduceLongBranchesToSingleVertices) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();
  ASSERT_THAT(octreeWithThreePoints -> getOctreeWithSIFT().getTreeDepth(), Eq(7));

  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNumberOfNodes(), Eq(7));
  std::string nodeNames[3] = {"F_0", "F_2", "F_4"};
  for (int i = 0; i < nodeNames -> size(); ++i) {
    Processors::Network::BayesNetworkNode node = network.getNode(nodeNames[i]);
    int depth = 0;
    while (node.getNumberOfChildren() != 0) {
      depth++;
      node = node.getChild();
    }
    ASSERT_THAT(depth, Eq(2));
  }
}

TEST_F(NetworkBuilderIntegrationTest, shouldHaveAtMostSameNumberOfLevelsAsOctree) {
  const int OCTREE_DEPTH = 7;
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();
  ASSERT_THAT(octreeWithThreePoints -> getOctreeWithSIFT().getTreeDepth(), Eq(OCTREE_DEPTH));

  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  std::string nodeNames[3] = {"F_0", "F_2", "F_4"};
  for (int i = 0; i < nodeNames -> size(); ++i) {
    Processors::Network::BayesNetworkNode node = network.getNode(nodeNames[i]);
    int depth = 0;
    while (node.getNumberOfChildren() != 0) {
      depth++;
      node = node.getChild();
    }
    ASSERT_THAT(depth, Le(OCTREE_DEPTH));
  }
}

TEST_F(NetworkBuilderIntegrationTest, shouldPointsFromSingleLeafInOctreeHaveSingleChild) {
  //TODO
}

TEST_F(NetworkBuilderIntegrationTest, shouldWriteNetworkToOutputPort) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderIntegrationTest, shouldInitializeHandlers) {
  networkBuilder -> prepareInterface();

  std::string handlers = networkBuilder -> listHandlers();
  ASSERT_THAT(handlers, Eq("onJointMultiplicity\nonNewModel\n"));
}
