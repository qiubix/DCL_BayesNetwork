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
  BayesNetwork network = getDefaultBayesNetwork();
  SOMEvaluation evaluator("evaluator");
  std::vector<int> indicesCollection;
  indicesCollection.push_back(1);
  evaluator.setInstance(indicesCollection);
  evaluator.setNetwork(&network);

  evaluator.evaluate();

  std::vector<std::string> featureNodeNames = network.getFeatureNodeNames();
  int numberOfActiveFeatures = 0;
  for (int i = 0; i < featureNodeNames.size(); i++) {
    if (network.getNodeEvidence(featureNodeNames[i]) == 1) {
      ++numberOfActiveFeatures;
    }
  }
  ASSERT_THAT(numberOfActiveFeatures, Eq(indicesCollection.size()));
}

TEST(SOMEvalationIntegrationTest, shouldReturn1WhenAllFeaturesWereMatched) {
  BayesNetwork network = getDefaultBayesNetwork();
  SOMEvaluation evaluator("evaluator");
  std::vector<int> indicesCollection;
  indicesCollection.push_back(0);
  indicesCollection.push_back(1);
  evaluator.setInstance(indicesCollection);
  evaluator.setNetwork(&network);

  evaluator.evaluate();

  ASSERT_THAT(evaluator.getNodeProbability(0), Eq(1));
}

TEST(SOMEvalationIntegrationTest, shouldReturn0WhenNoFeaturesWereMatched) {
  BayesNetwork network = getDefaultBayesNetwork();
  SOMEvaluation evaluator("evaluator");
  std::vector<int> indicesCollection;
  indicesCollection.push_back(2);
  indicesCollection.push_back(3);
  evaluator.setInstance(indicesCollection);
  evaluator.setNetwork(&network);

  evaluator.evaluate();

  ASSERT_THAT(evaluator.getNodeProbability(0), Eq(0));
}

TEST(SOMEvalationIntegrationTest, shouldReturnProperFractionWhenOnlySomeFeaturesWereMatched) {
  BayesNetwork network = getDefaultBayesNetwork();
  SOMEvaluation evaluator("evaluator");
  std::vector<int> indicesCollection;
  indicesCollection.push_back(1);
  indicesCollection.push_back(3);
  evaluator.setInstance(indicesCollection);
  evaluator.setNetwork(&network);

  evaluator.evaluate();

  ASSERT_THAT(evaluator.getNodeProbability(0), Eq(0.5));
}
