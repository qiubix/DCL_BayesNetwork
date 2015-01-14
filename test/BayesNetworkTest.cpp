#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/CreateNetworkWithSpacialDependencies/BayesNetwork.hpp"
#include "../src/Components/CreateNetworkWithSpacialDependencies/BayesNetworkNode.hpp"

using namespace Processors::Network;

class BayesNetworkTest : public Test {
  //protected:
  //  BayesNetwork mockNetwork;
};

TEST_F(BayesNetworkTest, shouldCreateEmptyNetwork)
{
  BayesNetwork network;
  DSL_network theNet = network.getNetwork();
  EXPECT_EQ(theNet.GetNumberOfNodes(), 0);
}

TEST_F(BayesNetworkTest, shouldGetNumberOfNodesInNetwork)
{
  BayesNetwork network;
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
  BayesNetwork newNetwork;
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
  BayesNetwork network;
  ASSERT_EQ(network.getNumberOfNodes(), 0);
  const int FIRST_NODE_ID = 0;
  const int SECOND_NODE_ID = 1;
  const char* FIRST_NODE_NAME = "V_0";
  const char* SECOND_NODE_NAME = "V_1";
  network.addVoxelNode(FIRST_NODE_ID);
  network.addVoxelNode(SECOND_NODE_ID);
  int firstNodeId = network.getNetwork().FindNode(FIRST_NODE_NAME);
  int secondNodeId = network.getNetwork().FindNode(SECOND_NODE_NAME);

  int numberOfChildren = network.getNetwork().NumChildren(firstNodeId);
  int numberOfParents = network.getNetwork().NumParents(secondNodeId);
  ASSERT_EQ(0, numberOfChildren);
  ASSERT_EQ(0, numberOfParents);

  int parentCPTSize = network.getNetwork().GetNode(firstNodeId)->Definition()->GetSize();
  int childCPTSize = network.getNetwork().GetNode(secondNodeId)->Definition()->GetSize();
  ASSERT_EQ(2, parentCPTSize);
  ASSERT_EQ(2, childCPTSize);

  network.addArc(FIRST_NODE_NAME, SECOND_NODE_NAME);

  numberOfChildren = network.getNetwork().NumChildren(firstNodeId);
  numberOfParents = network.getNetwork().NumParents(secondNodeId);
  EXPECT_EQ(1, numberOfChildren);
  EXPECT_EQ(1, numberOfParents);

  parentCPTSize = network.getNetwork().GetNode(firstNodeId)->Definition()->GetSize();
  childCPTSize = network.getNetwork().GetNode(secondNodeId)->Definition()->GetSize();
  EXPECT_EQ(2, parentCPTSize);
  EXPECT_EQ(4, childCPTSize);
}

//FIXME: analyze whether this test is necessary at all
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

//FIXME: analyze whether this test is necessary at all
TEST_F(BayesNetworkTest, shouldAddFeatureNodeToNetwork)
{
  /*
   * create network with some nodes
   * add feature node
   * check if name correct
   * check if node is in the right layer
   * check if node has proper CPT
   */
  BayesNetwork newNetwork;
  ASSERT_EQ(newNetwork.getNumberOfNodes(), 0);
  const int NODE_ID = 0;
  const char* NODE_NAME = "F_0";
  newNetwork.addFeatureNode(NODE_ID);
  int code = newNetwork.getNetwork().FindNode(NODE_NAME);
  ASSERT_NE(code, DSL_OUT_OF_RANGE);
  ASSERT_EQ(newNetwork.getNumberOfNodes(), 1);
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
  BayesNetwork simpleNetwork;
  ASSERT_EQ(simpleNetwork.getNumberOfNodes(), 0);

  const int PARENT_NODE_ID = 0;
  const char* PARENT_NODE_NAME = "V_0";
  simpleNetwork.addVoxelNode(PARENT_NODE_ID);
  int parentId = simpleNetwork.getNetwork().FindNode(PARENT_NODE_NAME);

  const int CHILD_NODE_ID = 1;
  const char* CHILD_NODE_NAME = "V_1";
  simpleNetwork.addVoxelNode(CHILD_NODE_ID);
  int childId = simpleNetwork.getNetwork().FindNode(CHILD_NODE_NAME);

  //network.addArc(PARENT_NODE_NAME, CHILD_NODE_NAME);

  std::vector<double> probabilities;
  probabilities.push_back(1.0);
  probabilities.push_back(0.0);
  simpleNetwork.fillCPT(PARENT_NODE_NAME, probabilities);

  DSL_network theNet = simpleNetwork.getNetwork();
  DSL_node* parentNode = theNet.GetNode(parentId);
  DSL_sysCoordinates parentCoordinates(*(parentNode->Definition()));
  DSL_idArray *theNames = parentNode->Definition()->GetOutcomesNames();
  parentCoordinates[0] = theNames->FindPosition("YES");
  parentCoordinates.GoToCurrentPosition();
  double probability = parentCoordinates.UncheckedValue();
  EXPECT_EQ(probability, 1.0);
  //FIXME: TODO: finish it!
}

TEST_F(BayesNetworkTest, shouldGetFirstRootNode)
{
  BayesNetwork network;
  network.addFeatureNode(0);
  BayesNetworkNode node = network.getNextRootNode();
  const bool NODE_NOT_VISITED = false;
  const std::string FIRST_ROOT_NODE_NAME = "F_0";
  ASSERT_EQ(node.isVisited(), NODE_NOT_VISITED);
  ASSERT_EQ(node.getName(), FIRST_ROOT_NODE_NAME);
}

TEST_F(BayesNetworkTest, shouldGetNextNotVisitedRootNode)
{
  BayesNetwork network;
  network.addFeatureNode(0);
  network.addFeatureNode(1);
  network.addFeatureNode(2);
  const bool NODE_NOT_VISITED = false;
  const bool NODE_VISITED = true;
  const std::string FIRST_ROOT_NODE_NAME = "F_0";
  const std::string SECOND_ROOT_NODE_NAME = "F_1";
  const std::string THIRD_ROOT_NODE_NAME = "F_2";

  BayesNetworkNode firstNode = network.getNextRootNode();
  EXPECT_EQ(firstNode.isVisited(), NODE_NOT_VISITED);
  EXPECT_EQ(firstNode.getName(), FIRST_ROOT_NODE_NAME);
  firstNode.visitNode();
  ASSERT_EQ(firstNode.isVisited(), NODE_VISITED);

  BayesNetworkNode secondNode = network.getNextRootNode();
  EXPECT_EQ(secondNode.isVisited(), NODE_NOT_VISITED);
  EXPECT_EQ(secondNode.getName(), SECOND_ROOT_NODE_NAME);
  secondNode.visitNode();
  ASSERT_EQ(secondNode.isVisited(), NODE_VISITED);

  BayesNetworkNode thirdNode = network.getNextRootNode();
  EXPECT_EQ(thirdNode.isVisited(), NODE_NOT_VISITED);
  EXPECT_EQ(thirdNode.getName(), THIRD_ROOT_NODE_NAME);

  //TODO: test for not visited nodes, whether it gets the same node
  //TODO: test for the last feature node
}

TEST_F(BayesNetworkTest, shouldGetNodeChild)
{
  BayesNetwork network;
  network.addFeatureNode(0);
  network.addFeatureNode(1);
  network.addVoxelNode(0);
  network.addArc("F_0", "V_0");
  network.addArc("F_1", "V_0");

  BayesNetworkNode firstNode = network.getNextRootNode();
  BayesNetworkNode secondNode = network.getNextRootNode();

  BayesNetworkNode childNode = network.getChild(firstNode);
  EXPECT_EQ(childNode.getName(), "V_0");
  childNode = network.getChild(secondNode);
  EXPECT_EQ(childNode.getName(), "V_0");
}
