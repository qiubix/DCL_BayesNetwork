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
