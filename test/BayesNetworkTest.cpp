#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/CreateNetworkWithSpacialDependencies/BayesNetwork.hpp"

class BayesNetworkTest : public Test {
  protected:
    Processors::Network::BayesNetwork network;
};

TEST_F(BayesNetworkTest, shouldTestNothing)
{
  EXPECT_TRUE(true);
}

TEST_F(BayesNetworkTest, shouldGetNumberOfNodesInNetwork)
{
  int numberOfNodes = network.getNumberOfNodes();
  EXPECT_EQ(numberOfNodes, 0);
}

