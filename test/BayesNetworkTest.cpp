#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkNode.hpp"

using namespace Processors::Network;

class BayesNetworkTest : public Test {
  //  BayesNetwork mockNetwork;

  public:
    BayesNetworkTest() {
      NODE_ID = 0;
      VOXEL_NODE_NAME = "V_0";
      FEATURE_NODE_NAME = "F_0";
      FIRST_NODE_ID = 0;
      SECOND_NODE_ID = 1;
      FIRST_NODE_NAME = "V_0";
      SECOND_NODE_NAME = "V_1";
      PARENT_NODE_ID = 0;
      PARENT_NODE_NAME = "V_0";
      CHILD_NODE_ID = 1;
      CHILD_NODE_NAME = "V_1";
      FIRST_ROOT_NODE_NAME = "F_0";
      SECOND_ROOT_NODE_NAME = "F_1";
      THIRD_ROOT_NODE_NAME = "F_2";
    }

    ~BayesNetworkTest() {}

    DSL_node* getNode(const char* NODE_NAME) {
      int nodeHandle = network->getNetwork().FindNode(NODE_NAME);
      return network->getNetwork().GetNode(nodeHandle);
    }

    int getCPTSize(const char* NODE_NAME) {
      int nodeHandle = network->getNetwork().FindNode(NODE_NAME);
      int CPTSize = network->getNetwork().GetNode(nodeHandle)->Definition()->GetSize();
      return CPTSize;
    }

    int getNumberOfChildren(const char* NODE_NAME) {
      int nodeHandle = network->getNetwork().FindNode(NODE_NAME);
      int numberOfChildren = network->getNetwork().NumChildren(nodeHandle);
      return numberOfChildren;
    }

    BayesNetwork* createEmptyNetwork() {
      return new BayesNetwork();
    }

  protected:
    int NODE_ID;
    const char* VOXEL_NODE_NAME;
    const char* FEATURE_NODE_NAME;
    int FIRST_NODE_ID;
    int SECOND_NODE_ID;
    const char* FIRST_NODE_NAME;
    const char* SECOND_NODE_NAME;
    int PARENT_NODE_ID;
    const char* PARENT_NODE_NAME;
    int CHILD_NODE_ID;
    const char* CHILD_NODE_NAME;
    std::string FIRST_ROOT_NODE_NAME;
    std::string SECOND_ROOT_NODE_NAME;
    std::string THIRD_ROOT_NODE_NAME;

    BayesNetwork* network;
};

TEST_F(BayesNetworkTest, shouldCreateEmptyNetwork)
{
  BayesNetwork network;
  EXPECT_TRUE(network.isEmpty());
}

TEST_F(BayesNetworkTest, shouldGetNumberOfNodesInNetwork)
{
  BayesNetwork network;
  int numberOfNodes = network.getNumberOfNodes();
  EXPECT_EQ(0, numberOfNodes);
}

TEST_F(BayesNetworkTest, shouldCheckIfNetworkHasNode)
{
  BayesNetwork newNetwork;
  ASSERT_FALSE(newNetwork.hasNode(VOXEL_NODE_NAME));
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
  BayesNetwork* newNetwork = createEmptyNetwork();
  ASSERT_EQ(0, newNetwork->getNumberOfNodes());

  newNetwork->addVoxelNode(NODE_ID);

  ASSERT_TRUE(newNetwork->hasNode(VOXEL_NODE_NAME));
  ASSERT_EQ(1, newNetwork->getNumberOfNodes());
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
  ASSERT_EQ(0, network.getNumberOfNodes());
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
  ASSERT_EQ(0, newNetwork.getNumberOfNodes());

  newNetwork.addFeatureNode(NODE_ID);

  ASSERT_TRUE(newNetwork.hasNode(FEATURE_NODE_NAME));
  ASSERT_EQ(1, newNetwork.getNumberOfNodes());
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
  ASSERT_EQ(0, simpleNetwork.getNumberOfNodes());

  simpleNetwork.addVoxelNode(PARENT_NODE_ID);
  int parentId = simpleNetwork.getNetwork().FindNode(PARENT_NODE_NAME);

  simpleNetwork.addVoxelNode(CHILD_NODE_ID);
  int childId = simpleNetwork.getNetwork().FindNode(CHILD_NODE_NAME);

  simpleNetwork.addArc(PARENT_NODE_NAME, CHILD_NODE_NAME);

  std::vector<double> probabilities;
  probabilities.push_back(0.3);
  probabilities.push_back(0.0);
  simpleNetwork.fillCPT(PARENT_NODE_NAME, probabilities);

  DSL_network theNet = simpleNetwork.getNetwork();
  DSL_node* parentNode = theNet.GetNode(parentId);
  DSL_sysCoordinates parentCoordinates(*(parentNode->Definition()));
  DSL_idArray *theNames = parentNode->Definition()->GetOutcomesNames();
  parentCoordinates[0] = theNames->FindPosition("YES");
  parentCoordinates.GoToCurrentPosition();
  double probability = parentCoordinates.UncheckedValue();
  EXPECT_EQ(0.3, probability);

  probabilities.push_back(0.4);
  probabilities.push_back(0.2);
  simpleNetwork.fillCPT(CHILD_NODE_NAME, probabilities);

  DSL_node* childNode = theNet.GetNode(childId);
  int childCPTSize = childNode->Definition()->GetSize();
  ASSERT_EQ(4, childCPTSize);
  ASSERT_EQ(2, childNode->Definition()->GetNumberOfOutcomes());
  DSL_sysCoordinates childCoordinates(*(childNode->Definition()));
  theNames = childNode->Definition()->GetOutcomesNames();
  childCoordinates[0] = theNames->FindPosition("YES");
  childCoordinates.GoToCurrentPosition();
  childCoordinates.GoFirst();
  probability = childCoordinates.UncheckedValue();
  //FIXME: this value is not correct, sth isn't working, because it should return 0.3
  //TODO: get whole CPT matrix and examine it
  EXPECT_EQ(0.5, probability);

  //FIXME: TODO: finish it!
}

TEST_F(BayesNetworkTest, shouldGetFirstRootNode)
{
  BayesNetwork network;
  network.addFeatureNode(0);

  BayesNetworkNode node = network.getNextRootNode();

  ASSERT_FALSE(node.isVisited());
  ASSERT_EQ(FIRST_ROOT_NODE_NAME, node.getName());
}

TEST_F(BayesNetworkTest, shouldCopyNodes)
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

/*
TEST_F(BayesNetworkTest, shouldGetTheSameNode)
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
  EXPECT_TRUE(secondNode.isVisited());
  EXPECT_EQ(FIRST_ROOT_NODE_NAME, secondNode.getName());
}
*/

TEST_F(BayesNetworkTest, shouldGetNextRootNode)
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

TEST_F(BayesNetworkTest, shouldGetNextNotVisitedRootNode)
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
  EXPECT_EQ("V_0", childNode.getName());
  childNode = network.getChild(secondNode);
  EXPECT_EQ("V_0", childNode.getName());
}
