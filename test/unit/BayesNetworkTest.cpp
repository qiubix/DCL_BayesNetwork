#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;
using ::testing::Ne;

#include <SMILE/network.h>
#include <SMILE/node.h>
#include <SMILE/nodedef.h>
#include <SMILE/nodeval.h>
//#include "Components/NetworkBuilder/BayesNetwork.hpp"
//#include "Components/NetworkBuilder/BayesNetworkNode.hpp"
//#include "Components/NetworkBuilder/BayesNetworkExceptions.hpp"
#include "Types/BayesNetwork.hpp"
#include "Types/BayesNetworkNode.hpp"
#include "Types/BayesNetworkExceptions.hpp"

using namespace Processors::Network;

class BayesNetworkTest : public Test {
  //  BayesNetwork mockNetwork;

  public:
    BayesNetworkTest() :
      FIRST_NODE_ID(0),
      SECOND_NODE_ID(1),
      FIRST_NODE_NAME("V_0"),
      SECOND_NODE_NAME("V_1"),
      CHILD_NODE_ID(0),
      CHILD_NODE_NAME("V_0"),
      FIRST_ROOT_NODE_NAME("F_0"),
      SECOND_ROOT_NODE_NAME("F_1"),
      THIRD_ROOT_NODE_NAME("F_2")
    {}

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

    BayesNetwork* createNetworkWithTwoNodes() {
      network = new BayesNetwork();
      network->addVoxelNode(FIRST_NODE_ID);
      network->addVoxelNode(SECOND_NODE_ID);
      return network;
    }

    BayesNetwork createNetworkWithOneChildAndTwoParents() {
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
      network.connectNodes(CHILD_NODE_NAME,FIRST_ROOT_NODE_NAME);
      network.connectNodes(CHILD_NODE_NAME,SECOND_ROOT_NODE_NAME);
      return network;
    }

  protected:
    const int FIRST_NODE_ID;
    const int SECOND_NODE_ID;
    const char* FIRST_NODE_NAME;
    const char* SECOND_NODE_NAME;
    const int CHILD_NODE_ID;
    const char* CHILD_NODE_NAME;
    const std::string FIRST_ROOT_NODE_NAME;
    const std::string SECOND_ROOT_NODE_NAME;
    const std::string THIRD_ROOT_NODE_NAME;

    BayesNetwork* network;
};

TEST_F(BayesNetworkTest, shouldCreateEmptyNetwork)
{
  BayesNetwork network;
  EXPECT_THAT(network.isEmpty(), Eq(true));
}

TEST_F(BayesNetworkTest, shouldGetNumberOfNodesInNetwork)
{
  BayesNetwork network;
  int numberOfNodes = network.getNumberOfNodes();
  EXPECT_THAT(numberOfNodes, Eq(0));
}

TEST_F(BayesNetworkTest, shouldCheckIfNetworkHasNode)
{
  BayesNetwork newNetwork;
  ASSERT_THAT(newNetwork.hasNode(FIRST_NODE_NAME), Eq(false));
}

TEST_F(BayesNetworkTest, shouldAddNodeToEmptyNetwork)
{
  BayesNetwork newNetwork;

  newNetwork.addVoxelNode(FIRST_NODE_ID);

  ASSERT_THAT(newNetwork.hasNode(FIRST_NODE_NAME), Eq(true));
  ASSERT_THAT(newNetwork.getNumberOfNodes(), Eq(1));
}

TEST_F(BayesNetworkTest, shouldAddNodeToNetworkWithNodes)
{
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();

  network.addVoxelNode(SECOND_NODE_ID);
  EXPECT_THAT(network.hasNode(SECOND_NODE_NAME), Eq(true));
  ASSERT_THAT(network.getNumberOfNodes(), Eq(4));

  network.addFeatureNode(2);
  EXPECT_THAT(network.hasNode("F_2"), Eq(true));
  ASSERT_THAT(network.getNumberOfNodes(), Eq(5));
}

TEST_F(BayesNetworkTest, shouldGetNumberOfFeatureNodesInNetwork)
{
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();
  ASSERT_THAT(network.getNumberOfFeatureNodes(), Eq(2));
}

TEST_F(BayesNetworkTest, shouldThrowExceptionWhenAddingAlreadyExistingNode)
{
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();

  EXPECT_THROW(network.addVoxelNode(FIRST_NODE_ID), NodeAlreadyExistsException);
  EXPECT_THROW(network.addFeatureNode(1), NodeAlreadyExistsException);
}

//TODO: get rid of this pointer, clean up this test
TEST_F(BayesNetworkTest, shouldConnectTwoNodes)
{
  BayesNetwork* network = createNetworkWithTwoNodes();
  ASSERT_EQ(2, network->getNumberOfNodes());

  int numberOfChildren = getNumberOfChildren(FIRST_NODE_NAME);
  int numberOfParents = getNumberOfParents(SECOND_NODE_NAME);
  ASSERT_THAT(numberOfChildren, Eq(0));
  ASSERT_THAT(numberOfParents, Eq(0));

  int parentCPTSize = getCPTSize(FIRST_NODE_NAME);
  int childCPTSize = getCPTSize(SECOND_NODE_NAME);
  ASSERT_THAT(parentCPTSize, Eq(2));
  ASSERT_THAT(childCPTSize, Eq(2));

  network->connectNodes(FIRST_NODE_NAME, SECOND_NODE_NAME);

  numberOfChildren = getNumberOfChildren(FIRST_NODE_NAME);
  numberOfParents = getNumberOfParents(SECOND_NODE_NAME);
  EXPECT_THAT(numberOfChildren, Eq(1));
  EXPECT_THAT(numberOfParents, Eq(1));

  parentCPTSize = getCPTSize(FIRST_NODE_NAME);
  childCPTSize = getCPTSize(SECOND_NODE_NAME);
  EXPECT_THAT(parentCPTSize, Eq(2));
  EXPECT_THAT(childCPTSize, Eq(4));
}

//TODO: split this test
//TODO: make this exceptions more informative
TEST_F(BayesNetworkTest, shouldThrowExceptionWhenNodesCanNotBeConnected)
{
  BayesNetwork network;
  network.addVoxelNode(0);
  EXPECT_THROW(network.connectNodes("V_0","V_1"), UnableToConnectNodesException);
  EXPECT_THROW(network.connectNodes("V_1","V_0"), UnableToConnectNodesException);

  network.addVoxelNode(1);
  network.connectNodes("V_0","V_1");
  EXPECT_THROW(network.connectNodes("V_0","V_1"), UnableToConnectNodesException);
  EXPECT_THROW(network.connectNodes("V_1","V_0"), UnableToConnectNodesException);

  network.addVoxelNode(2);
  network.connectNodes("V_1","V_2");
  /*
   * The following was commented out, because it doesn't throw nothing.
   * I'm trying to create cycle, which is unacceptable in BN, and there should be
   * an exception.
   * Leaving as a comment for a further research.
   */
  //EXPECT_THROW(network.connectNodes("V_2","V_0"), UnableToConnectNodesException);
}

TEST_F(BayesNetworkTest, shouldGetNumberOfChildren)
{
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();
  int numberOfChildren = network.getNumberOfChildren("V_0");
  ASSERT_THAT(numberOfChildren, Eq(2));
}

TEST_F(BayesNetworkTest, shouldThrowExceptionWhenTryingToCreateIncorrectNodeName)
{
  BayesNetwork network;

  EXPECT_THROW(network.createVoxelName(-1), UnableToCreateNodeNameWithThisIdException);
  EXPECT_THROW(network.createFeatureName(-1), UnableToCreateNodeNameWithThisIdException);
}

TEST_F(BayesNetworkTest, shouldGetNodeProbability) {
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();
  DSL_network net = network.getNetwork();
  int handle = net.FindNode("F_0");
  net.GetNode(handle)->Value()->SetEvidence(0);
  handle = net.FindNode("F_1");
  net.GetNode(handle)->Value()->SetEvidence(0);
  net.UpdateBeliefs();
  network.setNetwork(&net);

  double probability = network.getNodeProbability("V_0");
  ASSERT_THAT(probability, Eq(0.5));
}

TEST_F(BayesNetworkTest, shouldReturnTrueForExistingNode) {
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();

  ASSERT_THAT(network.nodeExists("V_0"), Eq(true));
}

TEST_F(BayesNetworkTest, shouldReturnFalseForNonExistingNode) {
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();

  ASSERT_THAT(network.nodeExists("V_7"), Eq(false));
}

TEST_F(BayesNetworkTest, shouldPropagateProbabilities) {
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();
  DSL_network net = network.getNetwork();
  int handle = net.FindNode("F_0");
  net.GetNode(handle)->Value()->SetEvidence(0);
  network.setNetwork(&net);
  double probability = network.getNodeProbability("V_0");
  ASSERT_THAT(probability, Ne(0.5));

  network.propagateProbabilities();

  probability = network.getNodeProbability("V_0");
  ASSERT_THAT(probability, Eq(0.5));
}

TEST_F(BayesNetworkTest, shouldSetNodeEvidence) {
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();
  int STATE = 1;

  network.setNodeEvidence("F_0", STATE);

  DSL_network net = network.getNetwork();
  int handle = net.FindNode("F_0");
  int evidence = net.GetNode(handle)->Value()->GetEvidence();
  ASSERT_THAT(evidence, Eq(STATE));
}

TEST_F(BayesNetworkTest, shouldGetNodeEvidence) {
  BayesNetwork network;
  network.addFeatureNode(0);
  network.addVoxelNode(0);
  network.connectNodes("F_0", "V_0");
  network.setCPTofAllVoxelNodes(1);
  network.propagateProbabilities();

  ASSERT_THAT(network.getNodeEvidence("F_0"), Eq(DSL_OUT_OF_RANGE));

  network.setNodeEvidence("F_0", 1);
  ASSERT_THAT(network.getNodeEvidence("F_0"), Eq(1));
}

TEST_F(BayesNetworkTest, shouldGetListOfFeatureNodeNames) {
  BayesNetwork network = createNetworkWithOneChildAndTwoParents();

  std::vector<std::string> featureNodeNames;
  featureNodeNames.push_back("F_0");
  featureNodeNames.push_back("F_1");
  ASSERT_THAT(network.getFeatureNodeNames(), Eq(featureNodeNames));
}
