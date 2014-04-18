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

    h_onJointMultiplicity.setup(this, &MapMultiplicity::onJointMultiplicity);
    registerHandler("onJointMultiplicity", &h_onJointMultiplicity);

    registerStream("in_jointCloud", &in_jointCloud);
    addDependency("onJointMultiplicity", &in_jointCloud);

    registerStream("out_jointMultiplicity", &out_jointMultiplicity);
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

void MapMultiplicity::onJointMultiplicity()
{
    LOG(LDEBUG) << "Extracting multiplicity of the joint cloud";
    pcl::PointCloud<PointXYZSIFT>::Ptr jointCloud = in_jointCloud.read();
    std::vector <int> jointMultiplicity;
    int featureMultiplicity;
    pcl::PointCloud<PointXYZSIFT>::iterator it = jointCloud->begin();
    while (it != jointCloud->end()) {
        featureMultiplicity = it->multiplicity;
        jointMultiplicity.push_back(featureMultiplicity);
        ++it;
    }
    LOG(LINFO) << "Number of joint model's features: " << jointMultiplicity.size();
    out_jointMultiplicity.write(jointMultiplicity);
}

}//: namespace Network
}//: namespace Processors
