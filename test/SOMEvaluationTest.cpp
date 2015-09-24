#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;
using ::testing::Return;

#include "../src/Components/SOMEvaluation/SOMEvaluation.hpp"
#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/AbstractNetwork.hpp"

using Processors::Network::SOMEvaluation;

class MockNetwork : public AbstractNetwork
{
public:
  MOCK_METHOD1(getNodeProbability, double(const std::string&));
  MOCK_METHOD0(clearEvidence, void());
};

TEST(SOMEvaluationTest, shouldDisplayDefaultProbabilityOnStart) {
  MockNetwork mockNetwork;
  EXPECT_CALL(mockNetwork, getNodeProbability("V_0"))
    .WillOnce(Return(0.5));

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);

  double hypothesisProbability = evaluator.getNodeProbability(0);
  EXPECT_THAT(hypothesisProbability, Eq(0.5));
}

TEST(SOMEvaluationTest, shouldClearAllEvidenceOnStart) {
  MockNetwork mockNetwork;
  EXPECT_CALL(mockNetwork, clearEvidence());

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);

  evaluator.evaluate();
}


/*
 * should set all evidence to NO on start
 * should call activate on all features in provided instance
 * should propagate probabilities to hypothesis node
 */
