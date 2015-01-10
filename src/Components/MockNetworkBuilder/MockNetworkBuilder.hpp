/*!
 * \file MockNetworkBuilder.hpp
 * \brief
 */

#ifndef MOCK_NETWORK_BUILDER_HPP_
#define MOCK_NETWORK_BUILDER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "OctreeContainers.hpp"
#include "BayesNetwork.hpp"

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Types/PointXYZSIFT.hpp>


namespace Processors {
namespace Network {

/*!
 * \class MockNetworkBuilder
 * \brief Class used to build Bayes network based on features multiplicity and spacial dependencies between them
 * \author Karol Katerżawa
 */
class MockNetworkBuilder: public Base::Component
{
public:
  /*!
   * Constructor.
   */
  MockNetworkBuilder(const std::string & name = "MockNetworkBuilder");

  /*!
   * Destructor
   */
  virtual ~MockNetworkBuilder();

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

  /// Event handlers
  Base::EventHandler <MockNetworkBuilder> h_onModels;
  Base::EventHandler2 h_onJointMultiplicity;
  Base::EventHandler2 h_onNewModel;

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
  unsigned int maxLeafContainerSize;
  int nextId;
  unsigned int numberOfVoxels;
  std::stack <OctreeBranchNode<OctreeContainerEmptyWithId>*> parentQueue;

  void buildNetwork();
  void addParentsToQueue(OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode);

  void exportNetwork();
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("MockNetworkBuilder", Processors::Network::MockNetworkBuilder)

#endif /* MOCK_NETWORK_BUILDER_HPP_ */

