/*!
 * \file MockFeatureMatcher.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "MockFeatureMatcher.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_representation.h>

#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

namespace Processors {
namespace Network {

class SIFTFeatureRepresentation: public pcl::DefaultFeatureRepresentation <PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
{
	using pcl::PointRepresentation<PointXYZSIFT>::nr_dimensions_;
	public:
	SIFTFeatureRepresentation ()
	{
		// Define the number of dimensions
		nr_dimensions_ = 128 ;
		trivial_ = false ;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointXYZSIFT &p, float * out) const
	{
		//This representation is only for determining correspondences (not for use in Kd-tree for example - so use only SIFT part of the point
		for (register int i = 0; i < 128 ; i++)
			out[i] = p.descriptor[i];//p.descriptor.at<float>(0, i) ;
		//std::cout << "SIFTFeatureRepresentation:copyToFloatArray()" << std::endl ;
	}
};


MockFeatureMatcher::MockFeatureMatcher(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello MockFeatureMatcher";
}

MockFeatureMatcher::~MockFeatureMatcher()
{
    LOG(LTRACE)<<"Good bye MockFeatureMatcher";
}

void MockFeatureMatcher::prepareInterface()
{
    LOG(LTRACE) << "MockFeatureMatcher::prepareInterface";

    h_onJointCloud.setup(this, &MockFeatureMatcher::onJointCloud);
    registerHandler("onJointCloud", &h_onJointCloud);
    registerStream("in_jointCloud", &in_jointCloud);
    addDependency("onJointCloud", &in_jointCloud);

    h_onInstances.setup(this, &MockFeatureMatcher::onInstances);
    registerHandler("onInstances", &h_onInstances);
    registerStream("in_instances", &in_instances);
    addDependency("onInstances", &in_instances);
//    addDependency("onInstances", &in_jointCloud);

    h_onInstance.setup(this, &MockFeatureMatcher::onInstance);
    registerHandler("onInstance", &h_onInstance);
    registerStream("in_instance", &in_instance);
    addDependency("onInstance", &in_instance);
//    addDependency("onInstance", &in_jointCloud);

    h_onInstanceCloud.setup(this, &MockFeatureMatcher::onInstanceCloud);
    registerHandler("onInstanceCloud", &h_onInstanceCloud);
    registerStream("in_instanceCloud", &in_instanceCloud);
    addDependency("onInstanceCloud", &in_instanceCloud);
//    addDependency("onInstanceCloud", &in_jointCloud);

    registerStream("out_featuresIndexes", &out_featuresIndexes);
}

bool MockFeatureMatcher::onInit()
{
    LOG(LTRACE) << "MockFeatureMatcher::initialize";
    return true;
}

bool MockFeatureMatcher::onFinish()
{
    LOG(LTRACE) << "MockFeatureMatcher::finish";
    return true;
}

bool MockFeatureMatcher::onStart()
{
    LOG(LTRACE) << "MockFeatureMatcher::onStart";
    return true;
}


bool MockFeatureMatcher::onStop()
{
    LOG(LTRACE) << "MockFeatureMatcher::onStop";
    return true;
}

void MockFeatureMatcher::onJointCloud()
{
		if(jointCloud == NULL) {
				jointCloud = in_jointCloud.read();
		}
}

void MockFeatureMatcher::onInstances()
{
    if(jointCloud == NULL) {
        return;
    }

    std::vector <AbstractObject*> instances = in_instances.read();
    instance = dynamic_cast<SIFTObjectModel*>(instances.at(0))->cloud_xyzsift;
    LOG(LDEBUG) << "Instance cloud size: " << instance -> size();
    matchFeatures();
}

void MockFeatureMatcher::onInstance()
{
    if(jointCloud == NULL) {
        return;
    }

    instance = dynamic_cast<SIFTObjectModel*>(in_instance.read())->cloud_xyzsift;
    LOG(LDEBUG) << "Instance cloud size: " << instance -> size();
    matchFeatures();
}

void MockFeatureMatcher::onInstanceCloud()
{
    if(jointCloud == NULL) {
        return;
    }

    instance = in_instanceCloud.read();
    LOG(LDEBUG) << "Instance cloud size: " << instance -> size();
    matchFeatures();
}

void MockFeatureMatcher::matchFeatures()
{
	LOG(LDEBUG) << "================= MockFeatureMatcher: determining correspondencies =================";


	LOG(LDEBUG) << "Number of matched features: " << featuresIndexes.size();
	out_featuresIndexes.write(featuresIndexes);
}

}//: namespace Network
}//: namespace Processors
