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
}


}//: namespace Network
}//: namespace Processors
