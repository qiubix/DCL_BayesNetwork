/*!
 * \file MapMultiplicity.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "MapMultiplicity.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

MapMultiplicity::MapMultiplicity(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello MapMultiplicity\n";
}

MapMultiplicity::~MapMultiplicity()
{
    LOG(LTRACE)<<"Good bye MapMultiplicity\n";
}

void MapMultiplicity::prepareInterface()
{
    LOG(LTRACE) << "MapMultiplicity::prepareInterface\n";

    h_onNewModel.setup(this, &MapMultiplicity::onNewModel);
    registerHandler("onNewModel", &h_onNewModel);
    h_onJointMultiplicity.setup(this, &MapMultiplicity::onJointMultiplicity);
    registerHandler("onJointMultiplicity", &h_onJointMultiplicity);

    registerStream("in_model", &in_model);
    addDependency("onNewModel", &in_model);
    registerStream("in_jointMultiplicity", &in_jointMultiplicity);
    addDependency("onJointMultiplicity", &in_jointMultiplicity);

    registerStream("out_network", &out_network);
}

bool MapMultiplicity::onInit()
{
    LOG(LTRACE) << "MapMultiplicity::initialize\n";

    return true;
}

bool MapMultiplicity::onFinish()
{
    LOG(LTRACE) << "MapMultiplicity::finish\n";

    return true;
}

bool MapMultiplicity::onStop()
{
    LOG(LTRACE) << "MapMultiplicity::onStop\n";

    return true;
}

bool MapMultiplicity::onStart()
{
    LOG(LTRACE) << "MapMultiplicity::onStart\n";

    return true;
}

void MapMultiplicity::onNewModel()
{
    LOG(LTRACE) << "MapMultiplicity::onNewModel\n";
}

void MapMultiplicity::onJointMultiplicity()
{
    LOG(LTRACE) << "MapMultiplicity::onJointMultiplicity\n";
}

}//: namespace Network
}//: namespace Processors
