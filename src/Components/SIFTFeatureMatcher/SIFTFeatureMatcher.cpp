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
//    addDependency("onInstances", &in_jointCloud);
    
    h_onInstance.setup(this, &SIFTFeatureMatcher::onInstance);
    registerHandler("onInstance", &h_onInstance);
    registerStream("in_instance", &in_instance);
    addDependency("onInstance", &in_instance);
//    addDependency("onInstance", &in_jointCloud);
    
    h_onInstanceCloud.setup(this, &SIFTFeatureMatcher::onInstanceCloud);
    registerHandler("onInstanceCloud", &h_onInstanceCloud);
    registerStream("in_instanceCloud", &in_instanceCloud);
    addDependency("onInstanceCloud", &in_instanceCloud);
//    addDependency("onInstanceCloud", &in_jointCloud);

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
    if(jointCloud == NULL) {
        return;
    }
    
    std::vector <AbstractObject*> instances = in_instances.read();
    instance = dynamic_cast<SIFTObjectModel*>(instances.at(0))->cloud_xyzsift;
    LOG(LDEBUG) << "Instance cloud size: " << instance -> size();
    matchFeatures();
}

void SIFTFeatureMatcher::onInstance()
{
    if(jointCloud == NULL) {
        return;
    }
    
    instance = dynamic_cast<SIFTObjectModel*>(in_instance.read())->cloud_xyzsift;
    LOG(LDEBUG) << "Instance cloud size: " << instance -> size();
    matchFeatures();
}

void SIFTFeatureMatcher::onInstanceCloud()
{
    if(jointCloud == NULL) {
        return;
    }
    
    instance = in_instanceCloud.read();
    LOG(LDEBUG) << "Instance cloud size: " << instance -> size();
    matchFeatures();
}

void SIFTFeatureMatcher::matchFeatures()
{
	LOG(LDEBUG) << "================= SIFTFeatureMatcher: determining correspondencies =================";
    
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst ;

	SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
	correst.setPointRepresentation(point_representation) ;
	correst.setInputSource(jointCloud) ;
	correst.setInputTarget(instance) ;
	correst.determineReciprocalCorrespondences(*correspondences) ;

	LOG(LDEBUG) << "Correspondences determined " << correspondences -> size();

	if ( correspondences -> size() > 20 ) {
		//ransac znalezienie blednych dopasowan
		pcl::Correspondences inliers ;
		pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
		sac.setInputSource(jointCloud) ;
		sac.setInputTarget(instance) ;
		sac.setInlierThreshold(0.001f) ;
		sac.setMaximumIterations(2000) ;
		sac.setInputCorrespondences(correspondences) ;
		sac.getCorrespondences(inliers);

		LOG(LDEBUG) << "Wrong matches found";

		//usuniecie blednych dopasowan
		pcl::Correspondences::iterator iter_inliers = inliers.begin();
		while(iter_inliers!=inliers.end()) {
			pcl::Correspondences::iterator iter_correspondences = correspondences->begin();
			while(iter_correspondences!=correspondences->end()) {
				if(iter_correspondences->index_query == iter_inliers->index_query) {
					iter_correspondences= correspondences->erase(iter_correspondences);
					break;
				}
				else {
					++iter_correspondences;
				}
			}
			++iter_inliers;	
		}
	}

	LOG(LINFO) << "Number of reciprocal correspondences: " << correspondences->size() << " out of " << jointCloud->size() << " keypoints";

	for(int i = 0; i< correspondences->size();i++){	
		if (correspondences->at(i).index_query >=jointCloud->size() ||
		        correspondences->at(i).index_match >=instance->size()){
			continue;
		}
        int featureIndex = correspondences -> at(i).index_query;
        featuresIndexes.push_back(featureIndex);
        LOG(LDEBUG) << "Index of matching feature: " << featureIndex;
	}
    
    LOG(LWARNING) << "Number of matched features: " << featuresIndexes.size();
    out_featuresIndexes.write(featuresIndexes);
}

}//: namespace Network
}//: namespace Processors
