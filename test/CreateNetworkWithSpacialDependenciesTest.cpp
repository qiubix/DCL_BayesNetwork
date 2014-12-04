#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/CreateNetworkWithSpacialDependencies/CreateNetworkWithSpacialDependencies.hpp"

class CreateNetworkWithSpacialDependenciesTest : public Test {
  protected:
    CreateNetworkWithSpacialDependencies component("name");
};

TEST_F(CreateNetworkWithSpacialDependenciesTest, shouldCreateComponentForTesting)
{
  component.prepareInterface();
  EXPECT_TRUE(true);
}
