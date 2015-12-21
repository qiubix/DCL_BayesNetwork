#include <gmock/gmock.h>
using ::testing::Eq;
using ::testing::_;
#include <gtest/gtest.h>
using ::testing::Test;
using ::testing::Return;
using ::testing::NiceMock;

#include "../src/Components/SOMEvaluation/SOMEvaluation.hpp"
#include "../src/Components/NetworkBuilder/BayesNetwork.hpp"
#include "../src/Components/NetworkBuilder/AbstractNetwork.hpp"

using Processors::Network::SOMEvaluation;

class MockNetwork : public AbstractNetwork
{
public:
  MOCK_METHOD1(getNodeProbability, double(const std::string&));
  MOCK_METHOD0(clearEvidence, void());
  MOCK_METHOD0(isEmpty, bool());
  MOCK_METHOD2(setNodeEvidence, void(const std::string&, int));
  MOCK_METHOD1(nodeExists, bool(const std::string&));
  MOCK_METHOD0(propagateProbabilities, void());
};

TEST(SOMEvaluationTest, shouldDisplayDefaultProbabilityOnStart) {
  NiceMock<MockNetwork> mockNetwork;
  EXPECT_CALL(mockNetwork, getNodeProbability("V_0"))
    .WillOnce(Return(0.5));

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);

  double hypothesisProbability = evaluator.getNodeProbability(0);
  EXPECT_THAT(hypothesisProbability, Eq(0.5));
}

TEST(SOMEvaluationTest, shouldClearAllEvidenceOnStart) {
  NiceMock<MockNetwork> mockNetwork;
  EXPECT_CALL(mockNetwork, clearEvidence());

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);

  evaluator.evaluate();
}

TEST(SOMEvaluationTest, shouldSetAllEvidenceToNoOnStart) {
  NiceMock<MockNetwork> mockNetwork;
  EXPECT_CALL(mockNetwork, nodeExists(_))
    .Times(3)
    .WillOnce(Return(true))
    .WillOnce(Return(true))
    .WillOnce(Return(false));
  EXPECT_CALL(mockNetwork, setNodeEvidence(_, 0)).Times(2);

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);

  evaluator.evaluate();
}

TEST(SOMEvaluationTest, shouldCallActivateOnAllFeaturesInProvidedInstance) {
  NiceMock<MockNetwork> mockNetwork;
  std::vector<int> instance;
  instance.push_back(1);
  instance.push_back(3);
  EXPECT_CALL(mockNetwork, setNodeEvidence("F_1", 1)).Times(1);
  EXPECT_CALL(mockNetwork, setNodeEvidence("F_3", 1)).Times(1);

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);
  evaluator.setInstance(instance);

  evaluator.evaluate();
}

TEST(SOMEvaluationTest, shouldPropagateProbabilitiesToHypothesisNode) {
  NiceMock<MockNetwork> mockNetwork;
  EXPECT_CALL(mockNetwork, propagateProbabilities()).Times(1);

  SOMEvaluation evaluator("evaluator");
  evaluator.setNetwork(&mockNetwork);

  evaluator.evaluate();
}
