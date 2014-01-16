/*!
 * \file CreateNetwork.hpp
 * \brief
 */

#ifndef CREATE_NETWORK_HPP_
#define CREATE_NETWORK_HPP_

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
 * \class CreateNetwork
 * \brief Class used to build Bayes network
 * \author Karol Katerżawa
 */
class CreateNetwork: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    CreateNetwork(const std::string & name = "");

    /*!
     * Destructor
     */
    virtual ~CreateNetwork();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:

    /// Input data stream
    Base::DataStreamIn< map<int,int> > in_model;
    Base::DataStreamIn< vector<int> > in_jointMultiplicity;

    /// Output data stream
    Base::DataStreamOut<DSL_network> out_network;

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
    Base::EventHandler <CreateNetwork> h_onNewModel;
    Base::EventHandler <CreateNetwork> h_onJointMultiplicity;

    /*!
     * Event handler function.
     */
    void onNewModel();
    void onJointMultiplicity();

private:
    DSL_network theNet;

    std::map <int, string> features;
    std::vector <int> jointMultiplicityVector;
    std::vector < std::map<int,int> > models;

    DSL_network getNetwork();

    std::string getNodeName(int nodeHandle);

    void addNode(const string name, const std::vector<string> outcomesNames, const std::vector<string> parentsNames);

    void setNodeCPT(const string name, vector<double> probabilities);

    void mapFeaturesNames();

    void buildNetwork();

    void setBaseNetworkCPTs();

    void setBaseFeaturesCPTs();

    void setBaseHypothesesCPTs();

    void loadNetwork();

    void exportNetwork();

};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("CreateNetwork", Processors::Network::CreateNetwork)

#endif /* CREATE_NETWORK_HPP_ */

