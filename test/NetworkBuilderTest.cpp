#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/NetworkBuilder/NetworkBuilder.hpp"

class NetworkBuilderTest : public Test {
  protected:
//    Processors::Network::NetworkBuilder* component = new Processors::Network::NetworkBuilder("name");
};

TEST_F(NetworkBuilderTest, shouldCreateComponentForTesting)
{
  Processors::Network::NetworkBuilder component("name");
  component.prepareInterface();
  EXPECT_TRUE(true);
}
