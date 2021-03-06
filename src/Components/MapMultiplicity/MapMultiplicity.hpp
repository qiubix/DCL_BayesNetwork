/*!
 * \file MapMultiplicity.hpp
 * \brief
 */

#pragma once
#ifndef MAP_MULTIPLICITY_HPP_
#define MAP_MULTIPLICITY_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

//#include <opencv2/core/core.hpp>
#include <vector>
//#include <map>

#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace Network {

/*!
 * \class MapMultiplicity
 * \brief Class used to build Bayes network
 * \author Karol Katerżawa
 */
class MapMultiplicity: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    MapMultiplicity(const std::string & name = "MapMultiplicity");

    /*!
     * Destructor
     */
    virtual ~MapMultiplicity();

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
REGISTER_COMPONENT("MapMultiplicity", Processors::Network::MapMultiplicity)

#endif /* MAP_MULTIPLICITY_HPP_ */

