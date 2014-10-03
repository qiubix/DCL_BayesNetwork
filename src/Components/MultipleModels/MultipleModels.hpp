/*!
 * \file MultipleModels.hpp
 * \brief
 */

#ifndef MULTIPLE_MODELS_HPP_
#define MULTIPLE_MODELS_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "OctreeContainers.hpp"

#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>

#include <Types/PointXYZSIFT.hpp>


namespace Processors {
namespace Network {

/*!
 * \class CreateNetwork
 * \brief Class used to build Bayes network
 * \author Karol Katerżawa
 */
class MultpleModels: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    MultpleModels(const std::string & name = "CreateNetwork");

    /*!
     * Destructor
     */
    virtual ~MultpleModels();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:

    /// Input data stream

    /// Output data stream

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

    /*!
     * Event handler function.
     */

private:
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("MultpleModels", Processors::Network::MultpleModels)

#endif /* MULTIPLE_MODELS_HPP_ */

