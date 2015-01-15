#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/CreateNetworkWithSpacialDependencies/Octree.hpp"
#include "../src/Components/CreateNetworkWithSpacialDependencies/OctreeNode.hpp"

using namespace Processors::Network;

class OctreeTest : public Test {
};

TEST_F(OctreeTest, shouldTestNothing) {
  ASSERT_TRUE(true);
}
