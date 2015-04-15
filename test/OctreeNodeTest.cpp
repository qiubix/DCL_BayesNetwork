#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/CPTManager.hpp"
#include "../src/Components/NetworkBuilder/CPTManagerExceptions.hpp"
#include "../src/Components/NetworkBuilder/BayesNetworkNode.hpp"

using namespace Processors::Network;

class OctreeNodeTest : public Test {
public:
  OctreeNodeTest() {};
  ~OctreeNodeTest() {};

protected:
  /* data */
};

TEST_F(OctreeNodeTest, shouldRunDummyTest)
{
  ASSERT_TRUE(true);
}
