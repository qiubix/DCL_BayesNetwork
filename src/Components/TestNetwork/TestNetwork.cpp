/*!
 * \file TestNetwork.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>
#include <Common/Timer.hpp>

#include "TestNetwork.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

TestNetwork::TestNetwork(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello TestNetwork";
}

TestNetwork::~TestNetwork()
{
  LOG(LTRACE)<<"Good bye TestNetwork";
}

void TestNetwork::prepareInterface()
{
  LOG(LTRACE) << "TestNetwork::prepareInterface";
  h_onNetwork.setup(this, &TestNetwork::onNetwork);
  registerHandler("onNetwork", &h_onNetwork);
  registerStream("in_network", &in_network);
  addDependency("onNetwork", &in_network);
}

bool TestNetwork::onInit()
{
  LOG(LTRACE) << "TestNetwork::initialize";
  return true;
}

bool TestNetwork::onFinish()
{
  LOG(LTRACE) << "TestNetwork::finish";
  return true;
}

bool TestNetwork::onStart()
{
  LOG(LTRACE) << "TestNetwork::onStart";
  return true;
}

bool TestNetwork::onStop()
{
  LOG(LTRACE) << "TestNetwork::onStop";
  return true;
}


}//: namespace Network
}//: namespace Processors
