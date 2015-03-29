#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/CPTManager.hpp"
#include "../src/Components/NetworkBuilder/CPTManagerExceptions.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkNode.hpp"

using namespace Processors::Network;

class CPTManagerTest : public Test {
public:
  CPTManagerTest() {
    network = new DSL_network();
    nextId = 0;
  }

  ~CPTManagerTest() {}

  DSL_node* createNodeWithoutCPT() {
    std::string nodeName = "Node" + nextId++;
    int nodeHandle = network->AddNode(DSL_CPT, nodeName.c_str());
    DSL_node* node = network->GetNode(nodeHandle);
    DSL_stringArray outcomes;
    outcomes.Add("true");
    outcomes.Add("false");
    node->Definition()->SetNumberOfOutcomes(outcomes);
    return node;
  }

  DSL_node* createNodeWithoutCPTWithParent() {
    DSL_node* parentNode = createNodeWithoutCPT();
    int parentNodeHandle = parentNode->Handle();
    DSL_node* childNode = createNodeWithoutCPT();
    int childNodeHandle = childNode->Handle();
    network->AddArc(parentNodeHandle, childNodeHandle);
    return childNode;
  }

  DSL_node* createNodeWithCPTOfSize2() {
    DSL_node* node = createNodeWithoutCPT();
    DSL_doubleArray theProbs;
    theProbs.SetSize(2);
    theProbs[0] = 0.8;
    theProbs[1] = 0.2;
    node->Definition()->SetDefinition(theProbs);
    return node;
  }

  DSL_node* createNodeWithCPTOfSize4() {
    DSL_node* parentNode = createNodeWithoutCPT();
    int parentNodeHandle = parentNode->Handle();
    DSL_node* childNode = createNodeWithoutCPT();
    int childNodeHandle = childNode->Handle();
    network->AddArc(parentNodeHandle, childNodeHandle);
    DSL_doubleArray theProbs;
    theProbs.SetSize(4);
    theProbs[0] = 0.4;
    theProbs[1] = 0.3;
    theProbs[2] = 0.2;
    theProbs[3] = 0.1;
    childNode->Definition()->SetDefinition(theProbs);
    return childNode;
  }

  std::vector<double> displayNodeCPT(DSL_node* node) {
    std::vector<double> probs;
    DSL_sysCoordinates coordinates(*node->Definition());
    while(true) {
      probs.push_back(coordinates.UncheckedValue());
      int position = coordinates.Next();
      if(position == DSL_OUT_OF_RANGE)
        break;
    }
    return probs;
  }

protected:
  DSL_network* network;
  int nextId;
};

TEST_F(CPTManagerTest, shouldDisplayCPTOfTheNodeWithoutParents)
{
  DSL_node* node = createNodeWithCPTOfSize2();
  ASSERT_EQ(2, node->Definition()->GetSize());
  CPTManager manager(node);

  std::vector<double> cpt = manager.displayCPT();

  //TODO: in C++11 it'll be much simpler
  const double probs[] = { 0.8,0.2 };
  std::vector<double> probabilities(probs, probs+sizeof(probs)/sizeof(double));
  ASSERT_EQ(probabilities, cpt);
}

TEST_F(CPTManagerTest, shouldDisplayCPTOfTheNodeWithParents)
{
  DSL_node* node = createNodeWithCPTOfSize4();
  ASSERT_EQ(4, node->Definition()->GetSize());
  CPTManager manager(node);

  std::vector<double> cpt = manager.displayCPT();

  //TODO: in C++11 it'll be much simpler
  const double probs[] = { 0.4, 0.3, 0.2, 0.1 };
  std::vector<double> probabilities(probs, probs+sizeof(probs)/sizeof(double));
  ASSERT_EQ(probabilities, cpt);
}

TEST_F(CPTManagerTest, shouldThrowExceptionWhenPassingVectorOfIncorrectSize)
{
  DSL_node* node = createNodeWithoutCPT();
  ASSERT_EQ(2, node->Definition()->GetSize());
  CPTManager manager(node);
  std::vector<double> probabilities;
  probabilities.push_back(0.8);

  ASSERT_THROW(manager.fillCPT(probabilities), DivergentCPTSizeException);

  probabilities.push_back(0.2);
  probabilities.push_back(0.1);

  ASSERT_THROW(manager.fillCPT(probabilities), DivergentCPTSizeException);
}

TEST_F(CPTManagerTest, shouldSetCPTOfTheNodeWithoutParents)
{
  DSL_node* node = createNodeWithoutCPT();
  ASSERT_EQ(2, node->Definition()->GetSize());
  CPTManager manager(node);
  std::vector<double> probabilities;
  probabilities.push_back(0.8);
  probabilities.push_back(0.2);

  manager.fillCPT(probabilities);
  ASSERT_EQ(probabilities, displayNodeCPT(node));
}

TEST_F(CPTManagerTest, shouldSetCPTOfTheNodeWithParent)
{
  DSL_node* node = createNodeWithoutCPTWithParent();
  ASSERT_EQ(4, node->Definition()->GetSize());
  CPTManager manager(node);
  std::vector<double> probabilities;
  probabilities.push_back(0.4);
  probabilities.push_back(0.3);
  probabilities.push_back(0.2);
  probabilities.push_back(0.1);

  manager.fillCPT(probabilities);
  ASSERT_EQ(probabilities, displayNodeCPT(node));
}
