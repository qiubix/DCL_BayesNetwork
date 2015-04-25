#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkNode.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkExceptions.hpp"

using namespace Processors::Network;

class BayesNetworkNodeTest : public Test {

public:
  BayesNetworkNodeTest() :
    FIRST_ROOT_NODE_NAME("F_0"),
    SECOND_ROOT_NODE_NAME("F_1"),
    THIRD_ROOT_NODE_NAME("F_2")
  {}

  ~BayesNetworkNodeTest() {}

protected:
  const std::string FIRST_ROOT_NODE_NAME;
  const std::string SECOND_ROOT_NODE_NAME;
  const std::string THIRD_ROOT_NODE_NAME;
};

TEST_F(BayesNetworkNodeTest, shouldGetFirstRootNode)
{
  BayesNetwork network;
  network.addFeatureNode(0);

  BayesNetworkNode node = network.getNextRootNode();

  ASSERT_FALSE(node.isVisited());
  ASSERT_EQ(FIRST_ROOT_NODE_NAME, node.getName());
}

//FIXME: this should belong to BayesNetworkNode test suite
TEST_F(BayesNetworkNodeTest, shouldCopyNodes)
{
  BayesNetwork network;
  network.addFeatureNode(0);
  BayesNetworkNode node = network.getNextRootNode();
  ASSERT_FALSE(node.isVisited());
  ASSERT_EQ("F_0", node.getName());
  node.visitNode();
  ASSERT_TRUE(node.isVisited());

  BayesNetworkNode newNode = node;
  ASSERT_EQ("F_0", newNode.getName());
  ASSERT_TRUE(newNode.isVisited());
  //TODO: test for other cases of copying
}

TEST_F(BayesNetworkNodeTest, shouldGetNextRootNode)
{
  BayesNetwork network;
  network.addFeatureNode(0);
  network.addFeatureNode(1);

  BayesNetworkNode firstNode = network.getNextRootNode();
  EXPECT_FALSE(firstNode.isVisited());
  EXPECT_EQ(FIRST_ROOT_NODE_NAME, firstNode.getName());
  network.visitNode(firstNode);
  ASSERT_TRUE(firstNode.isVisited());

  BayesNetworkNode secondNode = network.getNextRootNode();
  EXPECT_FALSE(secondNode.isVisited());
  EXPECT_EQ(SECOND_ROOT_NODE_NAME, secondNode.getName());
}

/*TODO: FIXME: split to multiple tests, see test_cases.md
 * - should return first root node
 * - should return next not visited node
 * - should return the same node if last root node still wasn't visited
 * - should stop at the last node
 * Probably it's a good idea to implement iterator for root nodes
 */
TEST_F(BayesNetworkNodeTest, shouldGetNextNotVisitedRootNode)
{
  BayesNetwork network;
  network.addFeatureNode(0);
  network.addFeatureNode(1);
  network.addFeatureNode(2);
  ASSERT_EQ(3, network.getNumberOfNodes());

  BayesNetworkNode firstNode = network.getNextRootNode();
  EXPECT_FALSE(firstNode.isVisited());
  EXPECT_EQ(FIRST_ROOT_NODE_NAME, firstNode.getName());
  network.visitNode(firstNode);
  ASSERT_TRUE(firstNode.isVisited());

  BayesNetworkNode secondNode = network.getNextRootNode();
  EXPECT_FALSE(secondNode.isVisited());
  EXPECT_EQ(SECOND_ROOT_NODE_NAME, secondNode.getName());

  //second node wasn't marked as visited
  BayesNetworkNode anotherNode = network.getNextRootNode();
  EXPECT_EQ(SECOND_ROOT_NODE_NAME, anotherNode.getName());

  network.visitNode(secondNode);
  ASSERT_TRUE(secondNode.isVisited());

  BayesNetworkNode thirdNode = network.getNextRootNode();
  EXPECT_FALSE(thirdNode.isVisited());
  EXPECT_EQ(THIRD_ROOT_NODE_NAME, thirdNode.getName());

  network.visitNode(thirdNode);

  //no more root nodes in network, so method getNextRootNode() returns last node
  BayesNetworkNode lastNode = network.getNextRootNode();
  EXPECT_TRUE(lastNode.isVisited());
  EXPECT_EQ(THIRD_ROOT_NODE_NAME, lastNode.getName());
}


TEST_F(BayesNetworkNodeTest, shouldGetNodeChild)
{
  BayesNetwork network;
  network.addFeatureNode(0);
  network.addFeatureNode(1);
  network.addVoxelNode(0);
  network.connectNodes("F_0", "V_0");
  network.connectNodes("F_1", "V_0");

  BayesNetworkNode firstNode = network.getNextRootNode();
  BayesNetworkNode secondNode = network.getNextRootNode();

  BayesNetworkNode childNode = network.getChild(firstNode);
  EXPECT_EQ("V_0", childNode.getName());
  childNode = network.getChild(secondNode);
  EXPECT_EQ("V_0", childNode.getName());
}
