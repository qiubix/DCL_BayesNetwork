/*!
 * \file SOMEvaluation.hpp
 * \brief
 */

#ifndef SOM_EVALUATION_HPP_
#define SOM_EVALUATION_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace Network {

/*!
 * \class SOMEvaluation
 * \brief Class used to evaluate instances basing on information about models' features multiplicity
 * \author Karol Katerżawa
 */
class SOMEvaluation: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    SOMEvaluation(const std::string & name = "SOMEvaluation");

    /*!
     * Destructor
     */
    virtual ~SOMEvaluation();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:
    ///Input data streams
    Base::DataStreamIn< std::vector<DSL_network> > in_networks;
    Base::DataStreamIn< std::vector<int> > in_instanceMatchedFeatures;

    //Output data streams
    Base::DataStreamOut< std::vector<double> > out_probabilities;

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
    Base::EventHandler <SOMEvaluation> h_onNetwork;
    Base::EventHandler <SOMEvaluation> h_onInstance;

    /*!
     * Event handler function.
     */
    void onNetwork();
    void onInstance();

private:
    std::map <int, string> features;
    std::vector <int> jointMultiplicityVector;
    std::vector <int> instance;
    std::vector <double> hypothesesProbabilities;

    std::vector<DSL_network> networks;
    DSL_network theNet;

    void evaluate();
    void deactivateFeatures();
    void activateMatchedFeatureNodes();
    void displayHypothesisProbability(int modelId = 0);
    int findFeatureNode(int nodeId);
    double getNodeProbability(int nodeId);
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMEvaluation", Processors::Network::SOMEvaluation)

#endif /* SOM_EVALUATION_HPP_ */

