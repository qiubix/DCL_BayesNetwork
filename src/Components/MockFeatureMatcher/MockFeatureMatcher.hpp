/*!
 * \file MockFeatureMatcher.hpp
 * \brief
 */

#ifndef MOCK_FEATURE_MATCHER_HPP_
#define MOCK_FEATURE_MATCHER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

//#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>

#include <vector>

#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTObjectModel.hpp>

namespace Processors {
namespace Network {

/*!
 * \class MockFeatureMatcher
 * \brief Class used to match SIFT features from instance to features in joint cloud.
 * \author Karol Katerżawa
 */
class MockFeatureMatcher: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    MockFeatureMatcher(const std::string & name = "MockFeatureMatcher");

    /*!
     * Destructor
     */
    virtual ~MockFeatureMatcher();

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

    /// Event handlers
    Base::EventHandler <MockFeatureMatcher> h_onJointCloud;
    Base::EventHandler <MockFeatureMatcher> h_onInstances;
    Base::EventHandler <MockFeatureMatcher> h_onInstance;
    Base::EventHandler <MockFeatureMatcher> h_onInstanceCloud;

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
REGISTER_COMPONENT("MockFeatureMatcher", Processors::Network::MockFeatureMatcher)

#endif /* MOCK_FEATURE_MATCHER_HPP_ */

