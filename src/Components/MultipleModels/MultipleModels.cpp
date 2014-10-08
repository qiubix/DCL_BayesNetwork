/*!
 * \file MultipleModels.cpp
 * \brief
 */

#include "MultipleModels.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

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

bool MultipleModels::onStart()
{
    LOG(LTRACE) << "MultipleModels::onStart\n";
    return true;
}

}//: namespace Network
}//: namespace Processors
