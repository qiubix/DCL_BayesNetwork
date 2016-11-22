/*!
 * \file BayesNetworkSink.cpp
 * \brief
 */

//#include <memory>
//#include <string>
#include <iostream>
//#include <opencv2/highgui/highgui.hpp>

#include "BayesNetworkSink.hpp"

#include "Logger.hpp"
#include <Types/BayesNetwork.hpp>

namespace Sinks {
namespace Network {

BayesNetworkSink::BayesNetworkSink(const std::string & name) :
    Base::Component(name)
{
  LOG(LTRACE)<<"Hello BayesNetworkSink\n";
}

BayesNetworkSink::~BayesNetworkSink() {
  LOG(LTRACE)<<"Good bye BayesNetworkSink\n";
}

void BayesNetworkSink::prepareInterface() {
  LOG(LTRACE) << "BayesNetworkSink::prepareInterface\n";
  registerStream("in_network", &in_network);
  registerHandler("onNewNetwork", boost::bind(&BayesNetworkSink::onNewNetwork, this));
  addDependency("onNewNetwork", &in_network);
}

bool BayesNetworkSink::onInit() {
  LOG(LTRACE) << "BayesNetworkSink::initialize\n";
  return true;
}

bool BayesNetworkSink::onFinish() {
  LOG(LTRACE) << "BayesNetworkSink::finish\n";
  return true;
}

bool BayesNetworkSink::onStop() {
  LOG(LTRACE) << "BayesNetworkSink::onStop\n";
  return true;
}

bool BayesNetworkSink::onStart() {
  LOG(LTRACE) << "BayesNetworkSink::onStart\n";
  return true;
}

void BayesNetworkSink::onNewNetwork() {
  setNetwork(in_network.read());
  display();
}

void BayesNetworkSink::setNetwork(Processors::Network::BayesNetwork* network) {
  this->network = network;
}

void BayesNetworkSink::display() {
  int numberOfFeatureNodes = network -> getNumberOfFeatureNodes();
  std::cout << "Number of feature nodes: " << numberOfFeatureNodes << std::endl;

  int totalNumberOfNodes = network -> getNumberOfNodes();
  std::cout << "Number of all nodes: " << totalNumberOfNodes << std::endl;
  LOG(LWARNING) << "END OF SEQUENCE\n";
}

}//: namespace Network
}//: namespace Sinks
