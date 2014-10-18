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
	registerStream("in_networks", &in_networks);
	
	// Register handlers
	h_onNetworks.setup(boost::bind(&MultipleModels::createGrid, this));
	registerHandler("createGrid", &h_onNetworks);

	// Register output data stream
	registerStream("out_probabilities", &out_probabilities);
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

bool MultipleModels::onStop()
{
    LOG(LTRACE) << "MultipleModels::onStop\n";
    return true;
}

void MultipleModels::createGrid()
{
  //TODO: creating grid of networks 
}

bool MultipleModels::onStart()
{
    LOG(LTRACE) << "MultipleModels::onStart\n";
    return true;
}

}//: namespace Network
}//: namespace Processors
