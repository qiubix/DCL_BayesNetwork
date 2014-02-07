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
    Base::DataStreamIn<DSL_network> in_network;
//	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr > in_instance;
//	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud_xyzsift;
    
//    Base::DataStreamIn< std::vector< std::map<int,int> > > in_models;
//    Base::DataStreamIn< std::vector<int> > in_jointMultiplicity;
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
    Base::EventHandler <SOMEvaluation> h_onModels;
    Base::EventHandler <SOMEvaluation> h_onNetwork;
    Base::EventHandler <SOMEvaluation> h_onInstance;

    /*!
     * Event handler function.
     */
    void onModels();
    void onNetwork();
    void onInstance();

private:
    std::map <int, string> features;
    std::vector <int> jointMultiplicityVector;
//    std::vector < std::map<int,int> > models;
    std::vector <int> instance;
    std::vector <double> hypothesesProbabilities;
    
    DSL_network theNet;
//    pcl::PointCloud<PointXYZSIFT>::Ptr instance;
    
    void evaluate();
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMEvaluation", Processors::Network::SOMEvaluation)

#endif /* SOM_EVALUATION_HPP_ */

