/*!
 * \file CreateNetworkWithSpacialDependencies.hpp
 * \brief
 */

#ifndef CREATE_NETWORK_WITH_SPACIAL_DEPENDENCIES_HPP_
#define CREATE_NETWORK_WITH_SPACIAL_DEPENDENCIES_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "OctreeContainers.hpp"

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
 * \class CreateNetworkWithSpacialDependencies
 * \brief Class used to build Bayes network based on features multiplicity and spacial dependencies between them
 * \author Karol Katerżawa
 */
class CreateNetworkWithSpacialDependencies: public Base::Component
{
public:
    /*!
     * Constructor.
     */
    CreateNetworkWithSpacialDependencies(const std::string & name = "CreateNetworkWithSpacialDependencies");

    /*!
     * Destructor
     */
    virtual ~CreateNetworkWithSpacialDependencies();

    /*!
     * Prepare data streams and handlers
     */
    void prepareInterface();

protected:

    /// Input data stream
    Base::DataStreamIn< std::vector< std::map<int,int> > > in_modelsMultiplicity;
    Base::DataStreamIn< std::vector<int> > in_jointMultiplicity;
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
    Base::EventHandler <CreateNetworkWithSpacialDependencies> h_onModels;
    Base::EventHandler <CreateNetworkWithSpacialDependencies> h_onJointMultiplicity;
		Base::EventHandler2 h_buildNetwork;

    /*!
     * Event handler function.
     */
    void buildNetwork();

private:
    DSL_network theNet;
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud;

    std::map <int, string> features;
    std::vector <int> jointMultiplicityVector;
    std::vector < std::map<int,int> > models;
    
    unsigned int branchNodeCount;
    unsigned int leafNodeCount;
    unsigned int maxLeafContainerSize;
    int nextId;
    
    void exportNetwork();

    void addHypothesisNode();
    
    void createChild(pcl::octree::OctreeNode* child, int parentId);
    void addVoxelNode(int id);
    string createVoxelName(int id);
    string createFeatureName(int id);

    void addNode(string name);
    
    void addArc(string parentName, string childName);
    void fillCPT(string name, std::vector<double> probabilities);
    int generateNext(std::string::iterator start, std::string::iterator end);

    std::string getNodeName(int nodeHandle);
    void mapFeaturesNames();
    void logLeafNodeContainerSize(pcl::octree::OctreeLeafNode< OctreeContainerPointIndicesWithId >* leaf_node);
    int sumMultiplicityInsideVoxel(pcl::octree::OctreeLeafNode< OctreeContainerPointIndicesWithId >* leaf_node);
    void logPoint(PointXYZSIFT p, int index);
    void loadNetwork();
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("CreateNetworkWithSpacialDependencies", Processors::Network::CreateNetworkWithSpacialDependencies)

#endif /* CREATE_NETWORK_WITH_SPACIAL_DEPENDENCIES_HPP_ */

