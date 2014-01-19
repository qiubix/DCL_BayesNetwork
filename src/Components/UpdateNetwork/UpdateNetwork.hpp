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
 * \author Karol Katerzawa
 */
class UpdateNetwork: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    UpdateNetwork(const std::string & name = "UpdateNetwork");

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

    Base::EventHandler <UpdateNetwork> h_onNetwork;
    Base::EventHandler <UpdateNetwork> h_onObservation;

    /*!
     * Event handler function.
     */
    void onNetwork();
    void onObservation();

    /// Input data stream
    Base::DataStreamIn<DSL_network> in_network;

    /// Output data stream
    Base::DataStreamOut< std::vector<double> > out_hypothesisProbabilities;

private:
    DSL_network theNet;

    void changeNodeCPT(const string nodeName, vector<double> probabilities);

    void observeNode(string observedNode, int observedState = 0);

    void clearEvidence();

};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("UpdateNetwork", Processors::Network::UpdateNetwork)

#endif /* UPDATE_NETWORK_HPP_ */

