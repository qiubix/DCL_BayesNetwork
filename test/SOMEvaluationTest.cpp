#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include "../src/Components/SOMEvaluation/SOMEvaluation.hpp"

TEST(SOMEvaluationTest, shouldDisplayDefaultProbabilityOnStart) {
  Processors::Network::SOMEvaluation evaluator;
  evaluator.theNet.AddNode(0,"V_0");

  double hypothesisProbability = evaluator.getNodeProbability(0);

  EXPECT_THAT(hypothesisProbability, Eq(0.5));
}
