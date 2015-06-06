#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

//#include <pcl/io/pcd_io.h>
#include "../src/Components/NetworkBuilder/NetworkBuilder.hpp"
#include "../src/Components/NetworkBuilder/NetworkBuilderExceptions.hpp"
#include "../src/Components/NetworkBuilder/CPTManager.hpp"

class NetworkBuilderTest : public Test {
public:
  NetworkBuilderTest() {
    networkBuilder = new Processors::Network::NetworkBuilder("name");
  }

  pcl::PointCloud<PointXYZSIFT>::Ptr getPointCloudWithOnePoint() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud -> width = 1;
    cloud -> height = 1;
    cloud -> points.resize(cloud -> width * cloud -> height);
    cloud -> points[0].x = 0.1;
    cloud -> points[0].y = 0.2;
    cloud -> points[0].z = 0.3;
    for(int i=0; i<128; i++) {
      cloud -> points[0].descriptor[i] = i;
    }
    return cloud;
  }

  pcl::PointCloud<PointXYZSIFT>::Ptr getPointCloudWithThreePoints() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud -> width = 3;
    cloud -> height = 1;
    cloud -> points.resize(cloud -> width * cloud -> height);
    cloud -> points[0].x = 0.0;
    cloud -> points[0].y = 0.0;
    cloud -> points[0].z = 0.0;
    cloud -> points[0].pointId = 0;
    cloud -> points[1].x = 1.0;
    cloud -> points[1].y = 1.0;
    cloud -> points[1].z = 1.0;
    cloud -> points[1].pointId = 2;
    cloud -> points[2].x = 0.0;
    cloud -> points[2].y = 0.0;
    cloud -> points[2].z = 1.0;
    cloud -> points[2].pointId = 4;
    for(int i=0; i<128; i++) {
      cloud -> points[0].descriptor[i] = i;
      cloud -> points[1].descriptor[i] = 128-i;
      cloud -> points[2].descriptor[i] = i*i;
    }
    return cloud;
  }

  Processors::Network::NetworkBuilder* networkBuilder;

};

TEST_F(NetworkBuilderTest, shouldThrowExceptionWhenBuildingFromEmptyCloud) {
  pcl::PointCloud<PointXYZSIFT>::Ptr emptyCloud(new pcl::PointCloud<PointXYZSIFT>);

  EXPECT_THROW(networkBuilder -> buildNetwork(emptyCloud), PointCloudIsEmptyException);
}

TEST_F(NetworkBuilderTest, shouldAddHypothesisNodeToNetwork) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloudWithOnePoint();

  networkBuilder -> buildNetwork(cloud);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.hasNode("V_0"), Eq(true));
}

TEST_F(NetworkBuilderTest, shouldBuildNetworkWithOnlyOneFeatureNode) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloudWithOnePoint();

  networkBuilder -> buildNetwork(cloud);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.hasNode("V_0"), Eq(true));
  ASSERT_THAT(network.hasNode("V_1"), Eq(true));
  ASSERT_THAT(network.hasNode("F_0"), Eq(true));
  ASSERT_THAT(network.getNumberOfNodes(), Eq(3));
}

TEST_F(NetworkBuilderTest, shouldBuildNetworkWithMultipleFeatureNodes) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloudWithThreePoints();

  networkBuilder -> buildNetwork(cloud);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNumberOfNodes(), Eq(7));
  ASSERT_THAT(network.hasNode("F_0"), Eq(true));
  ASSERT_THAT(network.hasNode("F_2"), Eq(true));
  ASSERT_THAT(network.hasNode("F_4"), Eq(true));
}

TEST_F(NetworkBuilderTest, shouldHaveTheSameNumberOfFeatureNodesAsCloudPoints) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloudWithThreePoints();

  networkBuilder -> buildNetwork(cloud);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNumberOfFeatureNodes(), Eq(3));
}

TEST_F(NetworkBuilderTest, shouldSetDefaultProbabilityValuesForFeatureNodes) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloudWithOnePoint();

  networkBuilder -> buildNetwork(cloud);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  Processors::Network::CPTManager manager = network.getNextRootNode().getNodeCPTManager();
  //TODO: in C++11 it'll be much simpler
  const double probs[] = { 0.5,0.5 };
  std::vector<double> probabilities(probs, probs+sizeof(probs)/sizeof(double));
  ASSERT_THAT(manager.displayCPT(), Eq(probabilities));
}

TEST_F(NetworkBuilderTest, shouldHaveOnlyOneChildNode) {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloudWithThreePoints();

  networkBuilder -> buildNetwork(cloud);

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
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloudWithThreePoints();

  networkBuilder -> buildNetwork(cloud);

  Processors::Network::BayesNetwork network = networkBuilder -> getNetwork();
  ASSERT_THAT(network.getNetwork().IsAcyclic(), Eq(1));
}

TEST_F(NetworkBuilderTest, shouldFillCPTsAcordingToNumberOfParents) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderTest, shouldHaveNodesWithUniqueNames) {
  EXPECT_TRUE(true);
}

TEST_F(NetworkBuilderTest, shouldWriteNetworkToOutputPort) {
  EXPECT_TRUE(true);
}
