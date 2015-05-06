/*!
 * \file NetworkBuilder.hpp
 * \brief
 */

#ifndef NETWORK_BUILDER_HPP
#define NETWORK_BUILDER_HPP

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "OctreeContainers.hpp"
#include "BayesNetwork.hpp"
#include "Octree.hpp"
#include "OctreeBranchNode.hpp"
#include "OctreeLeafNode.hpp"

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../../Types/PointXYZSIFT.hpp"


namespace Processors {
namespace Network {

/*!
 * \class NetworkBuilder
 * \brief Class used to build Bayes network based on features multiplicity and spacial dependencies between them
 * \author Karol Katerżawa
 */
class NetworkBuilder: public Base::Component
{
public:
  /*!
   * Constructor.
   */
  NetworkBuilder(const std::string & name = "NetworkBuilder");

  /*!
   * Destructor
   */
  virtual ~NetworkBuilder();

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
  Base::DataStreamOut<std::vector<DSL_network> > out_networks;

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
  void onNewModel();
  void onJointMultiplicity();

private:
  BayesNetwork network;
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
  std::stack <pcl::PointCloud<PointXYZSIFT>::Ptr> cloudQueue;

  std::map <int, string> features;
  std::vector <int> jointMultiplicityVector;
  std::vector < std::map<int,int> > models;

  unsigned int branchNodeCount;
  unsigned int leafNodeCount;
  unsigned int featureNodeCount;
  unsigned int maxLeafContainerSize;
  int nextId;
  unsigned int numberOfVoxels;
  std::stack <OctreeBranchNode> parentQueue;

  void buildNetwork();
  void addParentsToQueue(OctreeBranchNode branchNode);

  void createNode(OctreeNode* node);
  void createLeafNodeChildren(OctreeLeafNode leafNode);

  void connectNodeToNetwork(OctreeNode* child);

  void exportNetwork();

  void addHypothesisNode(OctreeBranchNode root, int modelId = 0);

  std::string getNodeName(int nodeHandle);
  void mapFeaturesNames();
  //void logLeafNodeContainerSize(pcl::octree::OctreeLeafNode< OctreeContainerPointIndicesWithId >* leaf_node);
  //int sumMultiplicityInsideVoxel(pcl::octree::OctreeLeafNode< OctreeContainerPointIndicesWithId >* leaf_node);
  void logPoint(PointXYZSIFT p, int index);
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("NetworkBuilder", Processors::Network::NetworkBuilder)

#endif /* NETWORK_BUILDER_HPP */

