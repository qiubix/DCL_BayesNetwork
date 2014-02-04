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
    
    h_onInstance.setup(this, &SIFTFeatureMatcher::onInstance);
    registerHandler("onInstance", &h_onInstance);
    
    registerStream("in_instanceCloud", &in_instanceCloud);
    addDependency("onInstance", &in_instanceCloud);

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

void SIFTFeatureMatcher::onInstance()
{
    pcl::PointCloud<PointXYZSIFT>::Ptr instance = in_instanceCloud.read();
}

}//: namespace Network
}//: namespace Processors
