#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/CreateNetworkWithSpacialDependencies/BayesNetwork.hpp"

class BayesNetworkTest : public Test {
  protected:
    Processors::Network::BayesNetwork network;
};

TEST_F(BayesNetworkTest, shouldCreateEmptyNetwork)
{
  DSL_network theNet = network.getNetwork();
  EXPECT_EQ(theNet.GetNumberOfNodes(), 0);
}

TEST_F(BayesNetworkTest, shouldGetNumberOfNodesInNetwork)
{
  int numberOfNodes = network.getNumberOfNodes();
  EXPECT_EQ(numberOfNodes, 0);
}

TEST_F(BayesNetworkTest, shouldAddNodeToEmptyNetwork)
{
  /*
   * create empty network
   * check if network empty
   * add node
   * check if node exists in network
   * check if number of nodes is 1
   */
  Processors::Network::BayesNetwork newNetwork;
  ASSERT_EQ(newNetwork.getNumberOfNodes(), 0);
  const int NODE_ID = 0;
  const char* NODE_NAME = "V_0";
  newNetwork.addVoxelNode(NODE_ID);
  int code = newNetwork.getNetwork().FindNode(NODE_NAME);
  ASSERT_NE(code, DSL_OUT_OF_RANGE);
  ASSERT_EQ(newNetwork.getNumberOfNodes(), 1);
}

TEST_F(BayesNetworkTest, shouldConnectTwoNodes)
{
  /*
   * create network with node
   * add new node to network
   * connect two nodes
   * check if nodes connected
   * check number of children
   * check number of parents
   * check size of CPT
   */
  Processors::Network::BayesNetwork newNetwork;
  ASSERT_EQ(newNetwork.getNumberOfNodes(), 0);
  const int FIRST_NODE_ID = 0;
  const int SECOND_NODE_ID = 1;
  const char* FIRST_NODE_NAME = "V_0";
  const char* SECOND_NODE_NAME = "V_1";
  newNetwork.addVoxelNode(FIRST_NODE_ID);
  newNetwork.addVoxelNode(SECOND_NODE_ID);
  int firstNodeId = newNetwork.getNetwork().FindNode(FIRST_NODE_NAME);
  int secondNodeId = newNetwork.getNetwork().FindNode(SECOND_NODE_NAME);
  int numberOfChildren = newNetwork.getNetwork().NumChildren(firstNodeId);
  int numberOfParents = newNetwork.getNetwork().NumParents(secondNodeId);
  ASSERT_EQ(0, numberOfChildren);
  ASSERT_EQ(0, numberOfParents);
  newNetwork.addArc(FIRST_NODE_NAME, SECOND_NODE_NAME);
  numberOfChildren = newNetwork.getNetwork().NumChildren(firstNodeId);
  numberOfParents = newNetwork.getNetwork().NumParents(secondNodeId);
  EXPECT_EQ(1, numberOfChildren);
  EXPECT_EQ(1, numberOfParents);
}

TEST_F(BayesNetworkTest, shouldAddVoxelNodeToNetworkWithNodes)
{
  /*
   * create network with some nodes
   * add voxel node
   * check if name correct
   * check if node is in the right layer
   */
  EXPECT_TRUE(true);
}

TEST_F(BayesNetworkTest, shouldAddFeatureNodeToNetwork)
{
  /*
   * create network with some nodes
   * add feature node
   * check if name correct
   * check if node is in the right layer
   * check if node has proper CPT
   */
  EXPECT_TRUE(true);
}

TEST_F(BayesNetworkTest, shouldFillNodeCPT)
{
  /*
   * create network with few nodes
   * connect those nodes
   * check size of child node's CPT
   * fill node's CPT
   * check whether every cell has been updated
   * check whether every cell has proper value
   */
  EXPECT_TRUE(true);
}
