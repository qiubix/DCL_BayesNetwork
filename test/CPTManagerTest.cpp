#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/CPTManager.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkNode.hpp"

using namespace Processors::Network;

class CPTManagerTest : public Test {
public:
  CPTManagerTest() {
    network = new DSL_network();
  }

  ~CPTManagerTest() {}

  DSL_node* createNodeWithoutCPT() {
    int nodeHandle = network->AddNode(DSL_CPT, "Node1");
    DSL_node* node = network->GetNode(nodeHandle);
    DSL_stringArray outcomes;
    outcomes.Add("true");
    outcomes.Add("false");
    node->Definition()->SetNumberOfOutcomes(outcomes);
    return node;
  }

  DSL_node* createNodeWithCPT() {
    DSL_node* node = createNodeWithoutCPT();
    DSL_doubleArray theProbs;
    theProbs.SetSize(2);
    theProbs[0] = 0.8;
    theProbs[1] = 0.2;
    node->Definition()->SetDefinition(theProbs);
    return node;
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
};

TEST_F(CPTManagerTest, shouldDisplayCPTOfTheNode)
{
  DSL_node* node = createNodeWithCPT();
  ASSERT_EQ(2, node->Definition()->GetSize());
  std::vector<double> p;
  p.push_back(0.8);
  p.push_back(0.3);
  ASSERT_EQ(p, displayNodeCPT(node));
}

TEST_F(CPTManagerTest, shouldSetCPTOfTheNodeWithoutChildren)
{
  DSL_node* node = createNodeWithoutCPT();
  ASSERT_EQ(2, node->Definition()->GetSize());
  CPTManager manager(node);
  ASSERT_TRUE(true);
}

TEST_F(CPTManagerTest, shouldSetCPTOfTheNodeWithChildren)
{
  ASSERT_TRUE(true);
}
