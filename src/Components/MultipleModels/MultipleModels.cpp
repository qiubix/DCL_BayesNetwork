/*!
 * \file MultipleModels.cpp
 * \brief
 */

#include "MultipleModels.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

MultipleModels::MultipleModels(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello MultipleModels\n";
}

MultipleModels::~MultipleModels()
{
  LOG(LTRACE)<<"Good bye MultipleModels\n";
}

void MultipleModels::prepareInterface()
{
  LOG(LTRACE) << "MultipleModels::prepareInterface\n";

  // Register data streams.
  registerStream("in_network", &in_network);

  // Register handlers
  h_onNetwork.setup(boost::bind(&MultipleModels::addNewNetwork, this));
  registerHandler("addNewNetwork", &h_onNetwork);
  addDependency("addNewNetwork", &in_network);

  // Register output data stream
  registerStream("out_networks", &out_networks);
}

bool MultipleModels::onInit()
{
  LOG(LTRACE) << "MultipleModels::initialize\n";
  return true;
}

bool MultipleModels::onFinish()
{
  LOG(LTRACE) << "MultipleModels::finish\n";
  return true;
}

bool MultipleModels::onStart()
{
  LOG(LTRACE) << "MultipleModels::onStart\n";
  return true;
}

bool MultipleModels::onStop()
{
  LOG(LTRACE) << "MultipleModels::onStop\n";
  return true;
}

void MultipleModels::createGrid()
{
  //TODO: creating grid of networks
  LOG(LTRACE) << "Creating grid of networks";
}

void MultipleModels::addNewNetwork()
{
  LOG(LTRACE) << "New network";
  DSL_network newNetwork = in_network.read();
  networks.push_back(newNetwork);
}

void MultipleModels::exportNetworks()
{
  LOG(LTRACE) << "Writting networks to output";
  out_networks.write(networks);
}

}//: namespace Network
}//: namespace Processors
