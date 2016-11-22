#include <gmock/gmock.h>
#include <Components/BayesNetworkSink/BayesNetworkSink.hpp>
#include <Types/BayesNetwork.hpp>

using Sinks::Network::BayesNetworkSink;
using Processors::Network::BayesNetwork;
using namespace testing;

class BayesNetworkSinkTest : public Test {
public:
  BayesNetworkSink sink;
};

TEST_F(BayesNetworkSinkTest, shouldCreateBayesNetworkSinkComponent) {
  ASSERT_THAT(sink.name(), Eq("BayesNetworkSink"));
}

TEST_F(BayesNetworkSinkTest, shouldInitializeStreams) {
  sink.prepareInterface();

  ASSERT_THAT(sink.getStream("in_network"), NotNull());
}

TEST_F(BayesNetworkSinkTest, shouldInitializeHandlers) {
  sink.prepareInterface();

  ASSERT_THAT(sink.getHandler("onNewNetwork"), NotNull());
}

TEST_F(BayesNetworkSinkTest, shouldImplementOnInitMethod) {
  ASSERT_THAT(sink.onInit(), Eq(true));
}

TEST_F(BayesNetworkSinkTest, shouldImplementOnFinishMethod) {
  ASSERT_THAT(sink.onFinish(), Eq(true));
}

TEST_F(BayesNetworkSinkTest, shouldImplementOnStartMethod) {
  ASSERT_THAT(sink.onStart(), Eq(true));
}

TEST_F(BayesNetworkSinkTest, shouldImplementOnStopMethod) {
  ASSERT_THAT(sink.onStop(), Eq(true));
}

//TEST_F(BayesNetworkSinkTest, shouldDisplayErrorIfNetworkIsNullOnNewNetwork) {
//  internal::CaptureStdout();
//
//  sink.onNewNetwork();
//
//  string output = internal::GetCapturedStdout();
//  ASSERT_THAT(output, Eq("Error! Network is NULL."));
//}

TEST_F(BayesNetworkSinkTest, shouldDisplayNetworkOnNewNetwork) {
  BayesNetwork network;
  network.addVoxelNode(0);
  network.addVoxelNode(1);
  network.addVoxelNode(2);
  network.addFeatureNode(0);
  network.addFeatureNode(1);
  network.connectNodes("F_0", "V_1");
  network.connectNodes("F_1", "V_2");
  network.connectNodes("V_1", "V_0");
  network.connectNodes("V_2", "V_0");
  sink.setNetwork(&network);

  internal::CaptureStdout();
  sink.display();

  string output = internal::GetCapturedStdout();
  ASSERT_THAT(output, HasSubstr("Number of feature nodes: 2"));
  ASSERT_THAT(output, HasSubstr("Number of all nodes: 5"));
  //TODO: get more info about network
//  ASSERT_THAT(output, Eq("[1, 1, 1, 1;\n  1, 1, 1, 1;\n  1, 1, 1, 1]\n"));
}
