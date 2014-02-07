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
#include "EventHandler2.hpp"

#include "../../../lib/SMILE/smile.h"
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Types/PointXYZSIFT.hpp>


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
    CreateNetwork(const std::string & name = "CreateNetwork");

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
    Base::DataStreamIn< std::vector< std::map<int,int> > > in_modelsMultiplicity;
    Base::DataStreamIn< vector<int> > in_jointMultiplicity;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud_xyzsift;

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
    Base::EventHandler <CreateNetwork> h_onModels;
    Base::EventHandler <CreateNetwork> h_onJointMultiplicity;
	Base::EventHandler2 h_cloud_xyzrgb_to_octree;

    /*!
     * Event handler function.
     */
    void onModels();
    void onJointMultiplicity();
	void cloud_xyzsift_to_octree();

private:
    DSL_network theNet;

    std::map <int, string> features;
    std::vector <int> jointMultiplicityVector;
    std::vector < std::map<int,int> > models;

    DSL_network getNetwork();

    std::string getNodeName(int nodeHandle);

    void addNode(string name);

    void setNodeCPT(string name, int numberOfParents);

    void mapFeaturesNames();

    void buildNetwork();

    void setBaseNetworkCPTs();

    void setBaseFeaturesCPTs();

    void setBaseHypothesesCPTs();

    void loadNetwork();

    void exportNetwork();
    
    //    void setVoxelCPT(pcl::octree::OctreeNode<PointXYZSIFT> node);
//    void addNodeParents(const std::string name, const std::vector<int> parentsId);
    void addArc(int parentId, int currentId);
    void addArc(string parentName, int currentId);
    int generateNext(std::string::iterator start, std::string::iterator end);
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("CreateNetwork", Processors::Network::CreateNetwork)

#endif /* CREATE_NETWORK_HPP_ */

