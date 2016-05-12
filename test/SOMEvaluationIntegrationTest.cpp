#include <gmock/gmock.h>
#include <Components/SOMEvaluation/SOMEvaluation.hpp>
#include "BayesNetworkGenerator.hpp"

using ::testing::Eq;
using ::testing::Test;

using Processors::Network::SOMEvaluation;

TEST(SOMEvalationIntegrationTest, shouldNotActivateAnyFeatureNodeInBayesNetworkForEmptyIndicesCollection) {
  //TODO
  getDefaultBayesNetwork();
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
