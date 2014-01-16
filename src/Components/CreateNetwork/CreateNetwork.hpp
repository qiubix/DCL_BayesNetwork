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
 * \author kkaterza
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

    DSL_network getNetwork();

    std::string getNodeName(int nodeHandle);

    void addNode(const string name, const std::vector<string> outcomesNames, const std::vector<string> parentsNames);

    void setNodeCPT(const string name, vector<double> probabilities);

    void mapFeaturesNames();

    void setBaseNetworkCPTs();

    void setBaseFeaturesCPTs();

    void setBaseHypothesesCPTs();

protected:

    /// Input data stream
    Base::DataStreamIn< vector<int> > in_model;
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

    /*!
     *
     */
    bool onStep();

    Base::EventHandler <CreateNetwork> h_onNewModel;
    Base::EventHandler <CreateNetwork> h_onJointMultiplicity;

    /*!
     * Event handler function.
     */
    void onNewModel();
    void onJointMultiplicity();
    void onNewImage();

private:
    cv::Mat img_uchar;

    DSL_network theNet;

    //FIXME: change mapping of features handlers (identifiers)
    std::map <int, string> features;
    std::vector <int> jointMultiplicityVector;
    std::vector < std::vector<int> > models;

    void initNetwork();

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

