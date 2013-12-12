/*!
 * \file UpdateNetwork.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <iostream>

#include "UpdateNetwork.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

UpdateNetwork::UpdateNetwork(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello UpdateNetwork\n";
}

UpdateNetwork::~UpdateNetwork()
{
    LOG(LTRACE)<<"Good bye UpdateNetwork\n";
}

void UpdateNetwork::prepareInterface()
{
    LOG(LTRACE) << "UpdateNetwork::prepareInterface\n";
}

bool UpdateNetwork::onInit()
{
    LOG(LTRACE) << "UpdateNetwork::initialize\n";

    return true;
}

bool UpdateNetwork::onFinish()
{
    LOG(LTRACE) << "UpdateNetwork::finish\n";

    return true;
}

bool UpdateNetwork::onStop()
{
    LOG(LTRACE) << "UpdateNetwork::onStop\n";

    return true;
}

bool UpdateNetwork::onStart()
{
    LOG(LTRACE) << "UpdateNetwork::onStart\n";

    return true;
}

bool UpdateNetwork::onStep()
{
    LOG(LTRACE) << "UpdateNetwork::onStep\n";

    return true;
}


void UpdateNetwork::onNewImage() {
    LOG(LTRACE) << "UpdateNetwork::onNewImage\n";
}

}//: namespace Network
}//: namespace Processors
