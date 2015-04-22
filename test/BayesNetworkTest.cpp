#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkNode.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkExceptions.hpp"

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

    int getNumberOfParents(const char* NODE_NAME) {
      int nodeHandle = network->getNetwork().FindNode(NODE_NAME);
      int numberOfChildren = network->getNetwork().NumParents(nodeHandle);
      return numberOfChildren;
    }

    BayesNetwork* createEmptyNetwork() {
      return new BayesNetwork();
    }

    BayesNetwork* createNetworkWithTwoNodes() {
      network = new BayesNetwork();
      network->addVoxelNode(FIRST_NODE_ID);
      network->addVoxelNode(SECOND_NODE_ID);
      return network;
    }

    BayesNetwork createNetworkWithOneParentAndTwoChildren() {
      BayesNetwork network;
      network.addVoxelNode(0);
      network.addFeatureNode(0);
      network.addFeatureNode(1);
      int parent = network.getNetwork().FindNode("V_0");
      int firstChild = network.getNetwork().FindNode("F_0");
      int secondChild = network.getNetwork().FindNode("F_1");
      //FIXME: use SMILE API, because connectNodes wasn't tested yet
      //getNetwork() returns a COPY of a network, so running AddArc will not change network stored in BN class
 //     network.getNetwork().AddArc(parent,firstChild);
 //     network.getNetwork().AddArc(parent,secondChild);
      network.connectNodes(PARENT_NODE_NAME,FIRST_ROOT_NODE_NAME);
      network.connectNodes(PARENT_NODE_NAME,SECOND_ROOT_NODE_NAME);
      return network;
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
  BayesNetwork newNetwork;

  newNetwork.addVoxelNode(NODE_ID);

  ASSERT_TRUE(newNetwork.hasNode(VOXEL_NODE_NAME));
  ASSERT_EQ(1, newNetwork.getNumberOfNodes());
}

TEST_F(BayesNetworkTest, shouldAddNodeToNetworkWithNodes)
{
  BayesNetwork network = createNetworkWithOneParentAndTwoChildren();

  network.addVoxelNode(SECOND_NODE_ID);
  EXPECT_TRUE(network.hasNode(SECOND_NODE_NAME));
  ASSERT_EQ(4, network.getNumberOfNodes());

  network.addFeatureNode(2);
  EXPECT_TRUE(network.hasNode("F_2"));
  ASSERT_EQ(5, network.getNumberOfNodes());
}

TEST_F(BayesNetworkTest, shouldThrowExceptionWhenAddingAlreadyExistingNode)
{
  BayesNetwork network = createNetworkWithOneParentAndTwoChildren();

  EXPECT_THROW(network.addVoxelNode(SECOND_NODE_ID), NodeAlreadyExistsException);
  EXPECT_THROW(network.addFeatureNode(2), NodeAlreadyExistsException);
}

//TODO: get rid of this pointer, clean up this test
TEST_F(BayesNetworkTest, shouldConnectTwoNodes)
{
  BayesNetwork* network = createNetworkWithTwoNodes();
  ASSERT_EQ(2, network->getNumberOfNodes());

  int numberOfChildren = getNumberOfChildren(FIRST_NODE_NAME);
  int numberOfParents = getNumberOfParents(SECOND_NODE_NAME);
  ASSERT_EQ(0, numberOfChildren);
  ASSERT_EQ(0, numberOfParents);

  int parentCPTSize = getCPTSize(FIRST_NODE_NAME);
  int childCPTSize = getCPTSize(SECOND_NODE_NAME);
  ASSERT_EQ(2, parentCPTSize);
  ASSERT_EQ(2, childCPTSize);

  network->connectNodes(FIRST_NODE_NAME, SECOND_NODE_NAME);

  numberOfChildren = getNumberOfChildren(FIRST_NODE_NAME);
  numberOfParents = getNumberOfParents(SECOND_NODE_NAME);
  EXPECT_EQ(1, numberOfChildren);
  EXPECT_EQ(1, numberOfParents);

  parentCPTSize = getCPTSize(FIRST_NODE_NAME);
  childCPTSize = getCPTSize(SECOND_NODE_NAME);
  EXPECT_EQ(2, parentCPTSize);
  EXPECT_EQ(4, childCPTSize);
}

//TODO: split this test
//TODO: make this exceptions more informative
//FIXME: missing test case: cycle
TEST_F(BayesNetworkTest, shouldThrowExceptionWhenNodesCanNotBeConnected)
{
  BayesNetwork network;
  network.addVoxelNode(0);
  EXPECT_THROW(network.connectNodes("V_0","V_1"), UnableToConnectNodesException);

  network.addVoxelNode(1);
  network.connectNodes("V_0","V_1");
  EXPECT_THROW(network.connectNodes("V_0","V_1"), UnableToConnectNodesException);
  EXPECT_THROW(network.connectNodes("V_1","V_0"), UnableToConnectNodesException);
}

TEST_F(BayesNetworkTest, shouldGetNumberOfChildren)
{
  BayesNetwork network = createNetworkWithOneParentAndTwoChildren();
  int numberOfChildren = network.getNumberOfChildren("V_0");
  ASSERT_EQ(2, numberOfChildren);
}

// **********************************************************************
//TODO: plough, it's redundant with CPTManager handling this issue
// **********************************************************************
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
  BayesNetwork simpleNetwork = *createNetworkWithTwoNodes();
  int parentId = simpleNetwork.getNetwork().FindNode(PARENT_NODE_NAME);
  int childId = simpleNetwork.getNetwork().FindNode(CHILD_NODE_NAME);

  simpleNetwork.connectNodes(PARENT_NODE_NAME, CHILD_NODE_NAME);

  std::vector<double> probabilities;
  probabilities.push_back(0.3);
  probabilities.push_back(0.1);
  simpleNetwork.fillCPT(PARENT_NODE_NAME, probabilities);

  DSL_network theNet = simpleNetwork.getNetwork();
  DSL_node* parentNode = theNet.GetNode(parentId);
  //FIXME: should have been Value, not Definition:
  DSL_sysCoordinates parentCoordinates(*parentNode->Definition());
  DSL_idArray *names = parentNode->Definition()->GetOutcomesNames();
  parentCoordinates[0] = 0; //names->FindPosition("YES");
  parentCoordinates.GoToCurrentPosition();
  double probability = parentCoordinates.UncheckedValue();
  EXPECT_EQ(0.3, probability);

  probabilities.push_back(0.4);
  probabilities.push_back(0.2);
  simpleNetwork.fillCPT(CHILD_NODE_NAME, probabilities);

  theNet = simpleNetwork.getNetwork();
  DSL_node* childNode = theNet.GetNode(childId);
  int childCPTSize = childNode->Definition()->GetSize();
  ASSERT_EQ(4, childCPTSize);
  ASSERT_EQ(2, childNode->Definition()->GetNumberOfOutcomes());
  //FIXME: should have been Value, not Definition:
  DSL_sysCoordinates childCoordinates(*childNode->Definition());
  names = childNode->Definition()->GetOutcomesNames();
  childCoordinates[0] = 0; //names->FindPosition("YES");
  childCoordinates.GoToCurrentPosition();
  probability = childCoordinates.UncheckedValue();
  //FIXME: this value is not correct, sth isn't working, because it's definition, not value
  EXPECT_EQ(0.3, probability);

  //FIXME: TODO: finish it!
}

// **********************************************************************
// TODO: add new fixture class, or move all tests below to new test suite
// **********************************************************************

TEST_F(BayesNetworkTest, shouldGetFirstRootNode)
{
  BayesNetwork network;
  network.addFeatureNode(0);

  BayesNetworkNode node = network.getNextRootNode();

  ASSERT_FALSE(node.isVisited());
  ASSERT_EQ(FIRST_ROOT_NODE_NAME, node.getName());
}

//FIXME: this should belong to BayesNetworkNode test suite
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

/*TODO: FIXME: split to multiple tests, see test_cases.md
 * - should return first root node
 * - should return next not visited node
 * - should return the same node if last root node still wasn't visited
 * - should stop at the last node
 * Probably it's a good idea to implement iterator for root nodes
 */
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
  network.connectNodes("F_0", "V_0");
  network.connectNodes("F_1", "V_0");

  BayesNetworkNode firstNode = network.getNextRootNode();
  BayesNetworkNode secondNode = network.getNextRootNode();

  BayesNetworkNode childNode = network.getChild(firstNode);
  EXPECT_EQ("V_0", childNode.getName());
  childNode = network.getChild(secondNode);
  EXPECT_EQ("V_0", childNode.getName());
}
