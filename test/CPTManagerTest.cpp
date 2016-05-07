#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "Components/NetworkBuilder/BayesNetwork.hpp"
#include "Components/NetworkBuilder/CPTManager.hpp"
#include "Components/NetworkBuilder/CPTManagerExceptions.hpp"
#include "Components/NetworkBuilder/BayesNetworkNode.hpp"

using namespace Processors::Network;

class CPTManagerTest : public Test {
public:
  CPTManagerTest() {
    network = new DSL_network();
    nextId = 0;
    PROBABILITY_VALUE = 0.6;
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

  DSL_node* createNodeWithoutCPTWithTwoParents() {
    DSL_node* firstParentNode = createNodeWithoutCPT();
    int firstParentNodeHandle = firstParentNode->Handle();
    DSL_node* secondParentNode = createNodeWithoutCPT();
    int secondParentNodeHandle = secondParentNode->Handle();
    DSL_node* childNode = createNodeWithoutCPT();
    int childNodeHandle = childNode->Handle();
    network->AddArc(firstParentNodeHandle, childNodeHandle);
    network->AddArc(secondParentNodeHandle, childNodeHandle);
    return childNode;
  }

  DSL_node* createNodeWithCPTOfSize2() {
    DSL_node* node = createNodeWithoutCPT();
    DSL_doubleArray probs;
    probs.SetSize(2);
    probs[0] = 0.8;
    probs[1] = 0.2;
    node->Definition()->SetDefinition(probs);
    return node;
  }

  DSL_node* createNodeWithCPTOfSize4() {
    DSL_node* parentNode = createNodeWithoutCPT();
    int parentNodeHandle = parentNode->Handle();
    DSL_node* childNode = createNodeWithoutCPT();
    int childNodeHandle = childNode->Handle();
    network->AddArc(parentNodeHandle, childNodeHandle);
    DSL_doubleArray probs;
    probs.SetSize(4);
    probs[0] = 0.4;
    probs[1] = 0.3;
    probs[2] = 0.2;
    probs[3] = 0.1;
    childNode->Definition()->SetDefinition(probs);
    return childNode;
  }

  DSL_node* createNodeWithObservedParent() {
    DSL_node* parentNode = createNodeWithoutCPT();
    int parentNodeHandle = parentNode->Handle();
    DSL_node* childNode = createNodeWithoutCPT();
    int childNodeHandle = childNode->Handle();
    network->AddArc(parentNodeHandle, childNodeHandle);
    DSL_doubleArray probs;
    probs.SetSize(4);
    probs[0] = 0.6;
    probs[1] = 0.4;
    probs[2] = 0.2;
    probs[3] = 0.8;
    childNode->Definition()->SetDefinition(probs);
    network->UpdateBeliefs();
    parentNode->Value()->SetEvidence(0);
    network->UpdateBeliefs();
    return childNode;
  }

protected:
  DSL_network* network;
  int nextId;
  double PROBABILITY_VALUE;
};

TEST_F(CPTManagerTest, shouldDisplayCPTOfTheNodeWithoutParents)
{
  DSL_node* node = createNodeWithCPTOfSize2();
  ASSERT_THAT(node->Definition()->GetSize(), Eq(2));
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
  ASSERT_THAT(node->Definition()->GetSize(), Eq(4));
  CPTManager manager(node);

  std::vector<double> cpt = manager.displayCPT();

  //TODO: in C++11 it'll be much simpler
  const double probs[] = { 0.4, 0.3, 0.2, 0.1 };
  std::vector<double> probabilities(probs, probs+sizeof(probs)/sizeof(double));
  ASSERT_THAT(cpt, Eq(probabilities));
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
  ASSERT_THAT(node->Definition()->GetSize(), Eq(2));
  CPTManager manager(node);
  std::vector<double> probabilities;
  probabilities.push_back(0.8);
  probabilities.push_back(0.2);

  manager.fillCPT(probabilities);
  ASSERT_THAT(manager.displayCPT(), Eq(probabilities));
}

TEST_F(CPTManagerTest, shouldSetCPTOfTheNodeWithParent)
{
  DSL_node* node = createNodeWithoutCPTWithParent();
  ASSERT_THAT(node->Definition()->GetSize(), Eq(4));
  CPTManager manager(node);
  std::vector<double> probabilities;
  probabilities.push_back(0.4);
  probabilities.push_back(0.3);
  probabilities.push_back(0.2);
  probabilities.push_back(0.1);

  manager.fillCPT(probabilities);
  ASSERT_THAT(manager.displayCPT(), Eq(probabilities));
}

//TODO: test for adding incorrect probabilities:
// not summing up to 1
// values are not from <0,1>
TEST_F(CPTManagerTest, shouldThrowExceptionWhenPassingIncorrectProbabilityValue)
{
  DSL_node* node = createNodeWithoutCPT();
  CPTManager manager(node);
  std::vector<double> probabilities;
  probabilities.push_back(1.8);
  probabilities.push_back(0.2);

  ASSERT_THROW(manager.fillCPT(probabilities), IncorrectProbabilityValueException);

  probabilities[0] = 0.8;
  probabilities[1] = -0.2;
  ASSERT_THROW(manager.fillCPT(probabilities), IncorrectProbabilityValueException);
}

TEST_F(CPTManagerTest, shouldDisplayNodeProbability)
{
  DSL_node* node = createNodeWithObservedParent();
  CPTManager manager(node);

  double probability = manager.getProbability();
  ASSERT_THAT(probability, Eq(PROBABILITY_VALUE));
}
