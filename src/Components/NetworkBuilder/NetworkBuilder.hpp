/*!
 * \file NetworkBuilder.hpp
 * \brief
 */

#pragma once
#ifndef NETWORK_BUILDER_HPP
#define NETWORK_BUILDER_HPP

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/AbstractNetwork.hpp"
//#include "BayesNetwork.hpp"
//#include "OctreeBranchNode.hpp"
//#include "OctreeLeafNode.hpp"

//#include <opencv2/core/core.hpp>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

//#include "Types/PointXYZSIFT.hpp"

//TODO: use forward declaration, create common base class for Octree::DepthFirstIterator
#include "Types/Octree.hpp"
#include "Types/DepthFirstIterator.hpp"

struct PointXYZSIFT;

namespace Processors {
namespace Network {

//class Octree;
class BayesNetwork;
class OctreeNode;
class OctreeLeafNode;
class OctreeBranchNode;
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

  void buildNetwork(Octree* octree);

  BayesNetwork getNetwork();

protected:

  /// Input data stream
  Base::DataStreamIn< std::vector< std::map<int,int> > > in_modelsMultiplicity;
  Base::DataStreamIn< std::vector<int> > in_jointMultiplicity;
//  Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud_xyzsift;
  Base::DataStreamIn< Octree* > in_octree;

  /// Output data stream
//  Base::DataStreamOut<DSL_network> out_network;
  //Base::DataStreamOut<std::vector<DSL_network> > out_networks;
  Base::DataStreamOut<std::vector<AbstractNetwork*> > out_networks;

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
  BayesNetwork* network;
//  std::stack <pcl::PointCloud<PointXYZSIFT>::Ptr> cloudQueue;
  std::stack <Octree* > octreeQueue;

  std::map <int, string> features;
  std::vector <int> jointMultiplicityVector;
  std::vector < std::map<int,int> > models;

  unsigned int branchNodeCount;
  unsigned int leafNodeCount;
  unsigned int featureNodeCount;
  unsigned int maxLeafContainerSize;
  int nextId;
  unsigned int numberOfVoxels;
  std::stack <OctreeBranchNode> parentStack;

  void addNodeToParentStack(OctreeBranchNode branchNode);

  void createNode(OctreeNode* node);
  void connectNodeToNetwork(string bayesParentNodeName);
  void createLeafNodeChildren(OctreeLeafNode leafNode, Octree* octree);


  void exportNetwork();

  void addHypothesisNode(DepthFirstIterator it, int modelId = 0);

  std::string getNodeName(int nodeHandle);
  void mapFeaturesNames();
  void logPoint(PointXYZSIFT p, int index);
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("NetworkBuilder", Processors::Network::NetworkBuilder)

#endif /* NETWORK_BUILDER_HPP */

