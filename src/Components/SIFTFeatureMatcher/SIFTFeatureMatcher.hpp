/*!
 * \file SIFTFeatureMatcher.hpp
 * \brief
 */

#ifndef SIFT_FEATURE_MATCHER_HPP_
#define SIFT_FEATURE_MATCHER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
//#include "Property.hpp"

//#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>

#include <vector>

#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTObjectModel.hpp>

namespace Processors {
namespace Network {

/*!
 * \class SIFTFeatureMatcher
 * \brief Class used to match SIFT features from instance to features in joint cloud. 
 * \author Karol Katerżawa
 */
class SIFTFeatureMatcher: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    SIFTFeatureMatcher(const std::string & name = "SIFTFeatureMatcher");

    /*!
     * Destructor
     */
    virtual ~SIFTFeatureMatcher();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:

    /// Input data stream
    Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_jointCloud;
    Base::DataStreamIn< std::vector<AbstractObject*> > in_instances;
    Base::DataStreamIn< AbstractObject* > in_instance;
    Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_instanceCloud;

    /// Output data stream
    Base::DataStreamOut< std::vector<int> > out_featuresIndexes;

    /*!
     * Connects source to given device.
     */
    bool onInit();

    /*!
     * Disconnect source from device, closes streams, etc.
     */
    bool onFinish();

    /*!
     * Start component
     */
    bool onStart();

    /*!
     * Stop component
     */
    bool onStop();

    /*!
     * Event handler function.
     */
    void onJointCloud();
    void onInstances();
    void onInstance();
    void onInstanceCloud();

private:

    pcl::PointCloud<PointXYZSIFT>::Ptr jointCloud;
    pcl::PointCloud<PointXYZSIFT>::Ptr instance;
    std::vector <int> featuresIndexes;
    
    void matchFeatures();

};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTFeatureMatcher", Processors::Network::SIFTFeatureMatcher)

#endif /* SIFT_FEATURE_MATCHER_HPP_ */

