#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/SOMEvaluation/SOMEvaluation.hpp"
#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"

TEST(SOMEvaluationTest, shouldDisplayDefaultProbabilityOnStart) {
  Processors::Network::BayesNetwork networkWrapper;
  networkWrapper.addVoxelNode(0);

  Processors::Network::SOMEvaluation evaluator("evaluator");
  evaluator.theNet = networkWrapper.getNetwork();

  double hypothesisProbability = evaluator.getNodeProbability(0);

  EXPECT_THAT(hypothesisProbability, Eq(0.5));
}
