/*!
 * \file SIFTFeatureMatcher.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "SIFTFeatureMatcher.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

SIFTFeatureMatcher::SIFTFeatureMatcher(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello SIFTFeatureMatcher";
}

SIFTFeatureMatcher::~SIFTFeatureMatcher()
{
    LOG(LTRACE)<<"Good bye SIFTFeatureMatcher";
}

void SIFTFeatureMatcher::prepareInterface()
{
    LOG(LTRACE) << "SIFTFeatureMatcher::prepareInterface";

    h_onJointCloud.setup(this, &SIFTFeatureMatcher::onJointCloud);
    registerHandler("onJointCloud", &h_onJointCloud);

    registerStream("in_jointCloud", &in_jointCloud);
    addDependency("onJointCloud", &in_jointCloud);
    
    h_onInstances.setup(this, &SIFTFeatureMatcher::onInstances);
    registerHandler("onInstances", &h_onInstances);
    
    registerStream("in_instances", &in_instances);
    addDependency("onInstances", &in_instances);

    registerStream("out_featuresIndexes", &out_featuresIndexes);
}

bool SIFTFeatureMatcher::onInit()
{
    LOG(LTRACE) << "SIFTFeatureMatcher::initialize";
    return true;
}

bool SIFTFeatureMatcher::onFinish()
{
    LOG(LTRACE) << "SIFTFeatureMatcher::finish";
    return true;
}

bool SIFTFeatureMatcher::onStart()
{
    LOG(LTRACE) << "SIFTFeatureMatcher::onStart";
    return true;
}


bool SIFTFeatureMatcher::onStop()
{
    LOG(LTRACE) << "SIFTFeatureMatcher::onStop";
    return true;
}

void SIFTFeatureMatcher::onJointCloud()
{
    if(jointCloud == NULL) {
		jointCloud = in_jointCloud.read();
    }
}

void SIFTFeatureMatcher::onInstances()
{
//    pcl::PointCloud<PointXYZSIFT>::Ptr instance = in_instances.read();
    std::vector <AbstractObject*> instances = in_instances.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr instance = dynamic_cast<SIFTObjectModel*>(instances.at(0))->cloud_xyzsift;
    
}

}//: namespace Network
}//: namespace Processors
