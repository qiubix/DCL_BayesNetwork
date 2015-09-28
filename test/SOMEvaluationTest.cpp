#include <gmock/gmock.h>
using ::testing::Eq;
using ::testing::_;
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
  MOCK_METHOD2(setNodeEvidence, void(const std::string&, int));
  MOCK_METHOD1(nodeExists, bool(const std::string&));
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

TEST(SOMEvaluationTest, shouldSetAllEvidenceToNoOnStart) {
  MockNetwork mockNetwork;
  EXPECT_CALL(mockNetwork, nodeExists(_)).Times(2).WillOnce(Return(true));
  EXPECT_CALL(mockNetwork, setNodeEvidence(_, 0)).Times(1);

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);

  evaluator.evaluate();
}


/*
 * should set all evidence to NO on start
 * should call activate on all features in provided instance
 * should propagate probabilities to hypothesis node
 */
