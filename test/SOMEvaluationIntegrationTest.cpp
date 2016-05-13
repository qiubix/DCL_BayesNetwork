#include <gmock/gmock.h>
#include <Components/SOMEvaluation/SOMEvaluation.hpp>
#include "BayesNetworkGenerator.hpp"
#include <vector>

using ::testing::Eq;
using ::testing::Test;

using Processors::Network::SOMEvaluation;
using Processors::Network::BayesNetwork;

TEST(SOMEvalationIntegrationTest, shouldNotActivateAnyFeatureNodeInBayesNetworkForEmptyIndicesCollection) {
  BayesNetwork network = getDefaultBayesNetwork();
  SOMEvaluation evaluator("evaluator");
  std::vector<int> emptyIndicesCollection;
  evaluator.setInstance(emptyIndicesCollection);
  evaluator.setNetwork(&network);

  evaluator.evaluate();

  ASSERT_THAT(network.getNodeEvidence("F_0"), Eq(0));
  ASSERT_THAT(network.getNodeEvidence("F_1"), Eq(0));
}

TEST(SOMEvalationIntegrationTest, shouldActivateSameNumberOfFeatureNodesInBayesNetworkAsNumberOfElementsInIndicesCollection) {
  //TODO
}

TEST(SOMEvalationIntegrationTest, shouldReturn1WhenAllFeaturesWereMatched) {
  //TODO
}

TEST(SOMEvalationIntegrationTest, shouldReturn0WhenNoFeaturesWereMatched) {
  //TODO
}

TEST(SOMEvalationIntegrationTest, shouldReturnProperFractionWhenOnlySomeFeaturesWereMatched) {
  //TODO
}
