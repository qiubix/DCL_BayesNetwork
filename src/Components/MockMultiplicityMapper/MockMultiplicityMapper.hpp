/*!
 * \file MockMultiplicityMapper.hpp
 * \brief
 */

#ifndef MOCK_MULTIPLICITY_MAPPER_HPP_
#define MOCK_MULTIPLICITY_MAPPER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <opencv2/core/core.hpp>
#include <vector>
#include <map>

#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace Network {

/*!
 * \class MockMultiplicityMapper
 * \brief Class used to build Bayes network
 * \author Karol Katerżawa
 */
class MockMultiplicityMapper: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    MockMultiplicityMapper(const std::string & name = "MockMultiplicityMapper");

    /*!
     * Destructor
     */
    virtual ~MockMultiplicityMapper();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:

    /// Input data stream
    Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_jointCloud;

    /// Output data stream
    Base::DataStreamOut< std::vector<int> > out_jointMultiplicity;

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
    Base::EventHandler <MockMultiplicityMapper> h_onJointMultiplicity;

    /*!
     * Event handler function.
     */
    void onJointMultiplicity();
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("MockMultiplicityMapper", Processors::Network::MockMultiplicityMapper)

#endif /* MOCK_MULTIPLICITY_MAPPER_HPP_ */

