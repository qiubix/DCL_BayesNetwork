/*!
 * \file UpdateNetwork.hpp
 * \brief
 */

#ifndef UPDATE_NETWORK_HPP_
#define UPDATE_NETWORK_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>


namespace Processors {
namespace Network {

/*!
 * \class UpdateNetwork
 * \brief Class used to build Bayes network
 * \author kkaterza
 */
class UpdateNetwork: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    UpdateNetwork(const std::string & name = "");

    /*!
     * Destructor
     */
    virtual ~UpdateNetwork();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:

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
     *
     */
    bool onStep();

    /*!
     * Event handler function.
     */
    void onNewImage();

    /// Input data stream
    Base::DataStreamIn<cv::Mat> in_img;

    Base::DataStreamIn<DSL_network> in_network;

    /// Output data stream - image with drawn blobs
    Base::DataStreamOut<cv::Mat> out_img;

private:
    cv::Mat img_uchar;

    DSL_network theNet;

    void observeNode(string observedNode, int observedState = 0);

};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("UpdateNetwork", Processors::Network::UpdateNetwork)

#endif /* UPDATE_NETWORK_HPP_ */

