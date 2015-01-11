#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/CreateNetworkWithSpacialDependencies/CreateNetworkWithSpacialDependencies.hpp"

class CreateNetworkWithSpacialDependenciesTest : public Test {
  protected:
//    Processors::Network::CreateNetworkWithSpacialDependencies* component = new Processors::Network::CreateNetworkWithSpacialDependencies("name");
};

TEST_F(CreateNetworkWithSpacialDependenciesTest, shouldCreateComponentForTesting)
{
  Processors::Network::CreateNetworkWithSpacialDependencies component("name");
  component.prepareInterface();
  EXPECT_TRUE(true);
}
