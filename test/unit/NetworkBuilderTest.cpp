#include <gmock/gmock.h>
using ::testing::Eq;
using ::testing::Le;
#include <gtest/gtest.h>
using ::testing::Test;

//#include <pcl/io/pcd_io.h>
#include "Types/BayesNetwork.hpp"
#include "Types/CPTManager.hpp"
#include <Types/Octree.hpp>
#include <Types/AbstractOctree.hpp>
#include <Types/AbstractNetwork.hpp>

#include "Components/NetworkBuilder/NetworkBuilder.hpp"
#include "Components/NetworkBuilder/NetworkBuilderExceptions.hpp"

#include "utils/PointCloudGenerator.hpp"

using Processors::Network::Octree;

class NetworkBuilderTest : public Test {
public:
  NetworkBuilderTest() {
    networkBuilder = new Processors::Network::NetworkBuilder("name");
  }

  Processors::Network::NetworkBuilder* networkBuilder;

};

//class MockOctree : public AbstractOctree {
//  virtual bool empty() = 0;
//  virtual int getNumberOfPoints() = 0;
//  virtual DepthFirstIterator depthBegin() = 0;
//  virtual DepthFirstIterator depthEnd() = 0;
//  virtual PointXYZSIFT getPoint(unsigned int id) = 0;
//  MOCK_METHOD0(empty, bool());
//  MOCK_METHOD0(getNumberOfPoints, int());
//
//};

//class MockNetwork : public AbstractNetwork
//{
//public:
//  MOCK_METHOD1(getNodeProbability, double(const std::string&));
//  MOCK_METHOD0(clearEvidence, void());
//  MOCK_METHOD0(isEmpty, bool());
//  MOCK_METHOD2(setNodeEvidence, void(const std::string&, int));
//  MOCK_METHOD1(nodeExists, bool(const std::string&));
//  MOCK_METHOD0(propagateProbabilities, void());
//};

TEST_F(NetworkBuilderTest, shouldThrowExceptionWhenBuildingFromEmptyCloud) {
  pcl::PointCloud<PointXYZSIFT>::Ptr emptyCloud(new pcl::PointCloud<PointXYZSIFT>);
  Octree octree(emptyCloud);

  EXPECT_THROW(networkBuilder -> buildNetwork(&octree), PointCloudIsEmptyException);
}

TEST_F(NetworkBuilderTest, shouldAddHypothesisNodeToNetwork) {
  Octree* octreeWithOnePoint = new Octree(getPointCloudWithOnePoint());
  octreeWithOnePoint -> init();

  networkBuilder -> buildNetwork(octreeWithOnePoint);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.hasNode("V_0"), Eq(true));
}

TEST_F(NetworkBuilderTest, shouldBuildNetworkWithOnlyOneFeatureNode) {
  Octree* octreeWithOnePoint = new Octree(getPointCloudWithOnePoint());
  octreeWithOnePoint -> init();

  networkBuilder -> buildNetwork(octreeWithOnePoint);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.hasNode("V_0"), Eq(true));
  ASSERT_THAT(network.hasNode("V_1"), Eq(true));
  ASSERT_THAT(network.hasNode("F_0"), Eq(true));
  ASSERT_THAT(network.getNumberOfNodes(), Eq(3));
}

TEST_F(NetworkBuilderTest, shouldBuildNetworkWithMultipleFeatureNodes) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();

  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNumberOfNodes(), Eq(7));
  ASSERT_THAT(network.hasNode("F_0"), Eq(true));
  ASSERT_THAT(network.hasNode("F_2"), Eq(true));
  ASSERT_THAT(network.hasNode("F_4"), Eq(true));
}

TEST_F(NetworkBuilderTest, shouldHaveTheSameNumberOfFeatureNodesAsPointsInOctree) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();

  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNumberOfFeatureNodes(), Eq(3));
}

TEST_F(NetworkBuilderTest, shouldSetDefaultProbabilityValuesForFeatureNodes) {
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

TEST_F(NetworkBuilderTest, shouldHaveOnlyOneChildNode) {
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

TEST_F(NetworkBuilderTest, shouldNotHaveCycles) {
  Octree* octreeWithThreePoints = new Octree(getPointCloudWithThreePoints());
  octreeWithThreePoints -> init();

  networkBuilder -> buildNetwork(octreeWithThreePoints);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNetwork().IsAcyclic(), Eq(1));
}

TEST_F(NetworkBuilderTest, shouldFillCPTsAcordingToNumberOfParents) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderTest, shouldHaveNodesWithUniqueNames) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderTest, shouldReduceLongBranchesToSingleVertices) {
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

TEST_F(NetworkBuilderTest, shouldHaveAtMostSameNumberOfLevelsAsOctree) {
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

TEST_F(NetworkBuilderTest, shouldPointsFromSingleLeafInOctreeHaveSingleChild) {
  //TODO
}

TEST_F(NetworkBuilderTest, shouldWriteNetworkToOutputPort) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderTest, shouldInitializeHandlers) {
  networkBuilder -> prepareInterface();

  std::string handlers = networkBuilder -> listHandlers();
  ASSERT_THAT(handlers, Eq("onJointMultiplicity\nonNewModel\n"));
}
