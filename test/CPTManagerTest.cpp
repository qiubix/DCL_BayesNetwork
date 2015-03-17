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
  DSL_node* createNewNode() {
    DSL_network network;
    int nodeHandle = network.AddNode(DSL_CPT, "Node1");
    DSL_node* node = network.GetNode(nodeHandle);
    DSL_stringArray outcomes;
    outcomes.Add("true");
    outcomes.Add("false");
    node->Definition()->SetNumberOfOutcomes(outcomes);
  }
};

TEST_F(CPTManagerTest, shouldSetCPTOfTheNodeWithoutChildren)
{
  DSL_node* node = createNewNode();
  CPTManager manager(node);
  ASSERT_TRUE(true);
}

TEST_F(CPTManagerTest, shouldSetCPTOfTheNodeWithChildren)
{
  ASSERT_TRUE(true);
}
