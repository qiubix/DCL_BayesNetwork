/*!
 * \file NetworkTester.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>
#include <Common/Timer.hpp>

#include "NetworkTester.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

NetworkTester::NetworkTester(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello NetworkTester";
  this->result = 0;
}

NetworkTester::~NetworkTester()
{
  LOG(LTRACE)<<"Good bye NetworkTester";
}

void NetworkTester::prepareInterface()
{
  LOG(LTRACE) << "NetworkTester::prepareInterface";
  h_onNetwork.setup(this, &NetworkTester::testNetwork);
  registerHandler("testNetwork", &h_onNetwork);
  registerStream("in_network", &in_network);
  addDependency("testNetwork", &in_network);

  registerStream("out_result", &out_result);
}

bool NetworkTester::onInit()
{
  LOG(LTRACE) << "NetworkTester::initialize";
  return true;
}

bool NetworkTester::onFinish()
{
  LOG(LTRACE) << "NetworkTester::finish";
  return true;
}

bool NetworkTester::onStart()
{
  LOG(LTRACE) << "NetworkTester::onStart";
  return true;
}

bool NetworkTester::onStop()
{
  LOG(LTRACE) << "NetworkTester::onStop";
  return true;
}

void NetworkTester::testNetwork()
{
  LOG(LTRACE) << "Testing network";
  network = in_network.read();
  displayAllNodeNamesInNetwork();
  observeOneFeatureNode();
  network.UpdateBeliefs();
}

void NetworkTester::displayAllNodeNamesInNetwork()
{
  int nodeId = network.GetFirstNode();
  while (nodeId != DSL_OUT_OF_RANGE) {
    string nodeName = getNodeName(nodeId);
    LOG(LWARNING) << "Node name: " << nodeName;
    nodeId = network.GetNextNode(nodeId);
  }
}

void NetworkTester::observeOneFeatureNode()
{
  int featureNode = findFeatureNode(0);
  if(featureNode != DSL_OUT_OF_RANGE) {
    observeNode(featureNode);
  }
}

string NetworkTester::getNodeName(int nodeId)
{
  DSL_node* node = network.GetNode(nodeId);
  DSL_nodeInfo info = node->Info();
  DSL_header header = info.Header();
  std::string nodeName(header.GetName());
  return nodeName;
}

int NetworkTester::findFeatureNode(int nodeId)
{
  std::stringstream ss;
  ss << "F_" << nodeId;
  std::string nodeName(ss.str());
  return network.FindNode(nodeName.c_str());
}

void NetworkTester::observeNode(int nodeId)
{
  network.GetNode(nodeId)->Value()->SetEvidence(0);
}

}//: namespace Network
}//: namespace Processors