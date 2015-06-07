/*!
 * \file NetworkBuilder.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <assert.h>

#include "NetworkBuilder.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"
#include "NetworkBuilderExceptions.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

NetworkBuilder::NetworkBuilder(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello NetworkBuilder\n";
  branchNodeCount = 0;
  leafNodeCount = 0;
  maxLeafContainerSize = 0;
  nextId = 0;
  numberOfVoxels = 0;
  featureNodeCount = 0;
}

NetworkBuilder::~NetworkBuilder()
{
  LOG(LTRACE)<<"Good bye NetworkBuilder\n";
}

void NetworkBuilder::prepareInterface()
{
  LOG(LTRACE) << "NetworkBuilder::prepareInterface";

  // Register data streams.
  //registerStream("in_cloud", &in_cloud_xyz);
  registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
  registerStream("in_jointMultiplicity", &in_jointMultiplicity);
  // Register handlers
  registerHandler("onNewModel", boost::bind(&NetworkBuilder::onNewModel, this));
  addDependency("onNewModel", &in_cloud_xyzsift);

  registerHandler("onJointMultiplicity", boost::bind(&NetworkBuilder::onJointMultiplicity, this));
  addDependency("onJointMultiplicity", &in_jointMultiplicity);

  //registerStream("out_network", &out_network);
  registerStream("out_networks", &out_networks);
}

bool NetworkBuilder::onInit()
{
  LOG(LTRACE) << "NetworkBuilder::initialize\n";
  return true;
}

bool NetworkBuilder::onFinish()
{
  LOG(LTRACE) << "NetworkBuilder::finish\n";
  return true;
}

bool NetworkBuilder::onStop()
{
  LOG(LTRACE) << "NetworkBuilder::onStop\n";
  return true;
}

void NetworkBuilder::onNewModel()
{
  LOG(LTRACE) << "On new model";
  pcl::PointCloud<PointXYZSIFT>::Ptr newCloud = in_cloud_xyzsift.read();
  cloudQueue.push(newCloud);
}

void NetworkBuilder::onJointMultiplicity()
{
  LOG(LTRACE) << "On joint multiplicity";
  jointMultiplicityVector = in_jointMultiplicity.read();
  if (cloudQueue.size() > 0) {
    LOG(LDEBUG) << "Size of cloudQueue: " << cloudQueue.size();
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = cloudQueue.top();
    cloudQueue.pop();
    buildNetwork(cloud);
  }
}

bool NetworkBuilder::onStart()
{
  LOG(LTRACE) << "NetworkBuilder::onStart\n";
  return true;
}

BayesNetwork NetworkBuilder::getNetwork() {
  return network;
}

void NetworkBuilder::buildNetwork(pcl::PointCloud<PointXYZSIFT>::Ptr cloud) {
  LOG(LDEBUG) << " #################### Building network ################### ";

  if( !network.isEmpty() ) {
    return;
  }

  if (cloud->empty()) {
    throw PointCloudIsEmptyException();
  }

  //TODO: move outside build() method
  Octree octree(cloud);
  octree.init();

  LOG(LTRACE) << "Creating iterators...";

  // Use depth-first iterator
  Octree::DepthFirstIterator dfIt = octree.depthBegin();
  const Octree::DepthFirstIterator dfItEnd = octree.depthEnd();

  LOG(LTRACE) << "Creating nodes:";

  // Root node
  addHypothesisNode(octree.depthBegin());
  ++dfIt;

  for (;dfIt != dfItEnd; ++dfIt) {
    LOG(LDEBUG) << "========= Another node in depth search =========";
    OctreeNode node = *dfIt;
    if (node.getNodeType() == OCTREE_LEAF_NODE) {
      LOG(LDEBUG) << "Entering octree leaf node.";
      OctreeLeafNode leafNode(node);
      createNode(&leafNode);
      createLeafNodeChildren(leafNode, cloud);
    }
    else if (node.getNodeType() == OCTREE_BRANCH_NODE) {
      LOG(LDEBUG) << "Entering octree branch node.";
      OctreeBranchNode branchNode(node);
      if(branchNode.hasOnlyOneChild()) {
        LOG(LDEBUG) << "Skipping octree node, that has only one child";
        continue;
      }
      else {
        LOG(LDEBUG) << "Node has multiple children, adding to Bayes network";
        createNode(&branchNode);
        addNodeToParentStack(branchNode);
        ++branchNodeCount;
      }
    }
  }
  network.setCPTofAllVoxelNodes(numberOfVoxels);

  exportNetwork();
}

void NetworkBuilder::createNode(OctreeNode* node)
{
  LOG(LDEBUG) << "Creating node: " << nextId;
  node->setId(nextId);
  network.addVoxelNode(nextId);
  string bayesParentNodeName = network.createVoxelName(nextId);
  ++numberOfVoxels;
  ++nextId;

  if (bayesParentNodeName == "V_0") {
    return;
  }
  else {
    connectNodeToNetwork(bayesParentNodeName);
  }
}

void NetworkBuilder::connectNodeToNetwork(string bayesParentNodeName)
{
  OctreeBranchNode parent(parentStack.top());
  int parentId = parent.getId();
  string bayesChildNodeName = network.createVoxelName(parentId);
  parentStack.pop();
  network.connectNodes(bayesParentNodeName, bayesChildNodeName);
}

void NetworkBuilder::addNodeToParentStack(OctreeBranchNode branchNode)
{
  LOG(LDEBUG) << "Adding parents to stack";
  if(!branchNode.hasOnlyOneChild() || branchNode.getId() == 0) {
    int numberOfChildren = branchNode.getNumberOfChildren();
    for (int i=0; i<numberOfChildren; i++) {
      parentStack.push(branchNode);
    }
    LOG(LDEBUG) << "Current node's children quantinty: " << numberOfChildren;
    LOG(LTRACE) << "Parent stack size: " << parentStack.size();
  }
}

void NetworkBuilder::createLeafNodeChildren(OctreeLeafNode leafNode, pcl::PointCloud<PointXYZSIFT>::Ptr cloud)
{
  LOG(LTRACE) << "----- Creating leaf node children -----";

  int parentId = leafNode.getId();
  string parentName = network.createVoxelName(parentId);
  int childrenCounter = leafNode.getNumberOfChildren();

  if (childrenCounter > maxLeafContainerSize) {
    maxLeafContainerSize = childrenCounter;
  }

  // Iterate through container elements, i.e. cloud points.
  std::vector<int> point_indices = leafNode.getPointIndices();
  LOG(LTRACE) << "point indices size: " << point_indices.size();

  LOG(LTRACE) << "Add all children:";

  //FIXME: Change the way coefficients are calculated
  for(unsigned int i=0; i<childrenCounter; i++)
  {
    LOG(LTRACE) << "Creating child number " << i;
    PointXYZSIFT p = cloud->at(point_indices[i]);
    logPoint(p, point_indices[i]);
    int featureId = p.pointId;
    network.addFeatureNode(featureId);
    string featureName = network.createFeatureName(featureId);
    network.connectNodes(featureName, parentName);
    ++featureNodeCount;
  }//: for points

  LOG(LTRACE) << "voxel ID: " << parentId;
  LOG(LTRACE) << "voxel name: " << network.createVoxelName(parentId);
  LOG(LTRACE) << "children count: " <<childrenCounter;
  leafNodeCount++;
}

void NetworkBuilder::exportNetwork()
{
  LOG(LWARNING) << "Branch node quantity: " << branchNodeCount;
  LOG(LWARNING) << "Leaf node quantity: " << leafNodeCount;
  LOG(LWARNING) << "Voxel node quantity: " << numberOfVoxels;
  LOG(LWARNING) << "Feature node quantity: " << featureNodeCount;
  LOG(LWARNING) << "maxLeafContainerSize: " << maxLeafContainerSize;

  LOG(LDEBUG) << "before writing network to file";
  network.exportNetworkToFile();
  LOG(LDEBUG) << "after writing network to file";
  std::vector<DSL_network> networks;
  networks.push_back(network.getNetwork());
  out_networks.write(networks);
  //out_network.write(network.getNetwork());
}

void NetworkBuilder::addHypothesisNode(Octree::DepthFirstIterator it, int modelId)
{
  LOG(LDEBUG) << "Creating hypothesis node. Model id: " << modelId;
  /*
   * FIXME: hardcoded modelId! Works only for one model.
   * For more models it'll probably cause conflicts with voxel nodes, which are enumerated with IDs > 0.
   * Possible solution: hypothesis node name may start with H_
   */
  OctreeNode node = *it;
  if(node.getNodeType() == OCTREE_BRANCH_NODE) {
    OctreeBranchNode root(*it);
    createNode(&root);
    addNodeToParentStack(root);
    ++branchNodeCount;
  }
  else {
    LOG(LDEBUG) << "Error creating hypothesis node. First node is not a branch node!";
    return;
  }
}

string NetworkBuilder::getNodeName(int nodeHandle)
{
  return features[nodeHandle];
}

void NetworkBuilder::logPoint(PointXYZSIFT p, int index)
{
  LOG(LTRACE) << "Point index = " << index;
  LOG(LTRACE) << "p.x = " << p.x << " p.y = " << p.y << " p.z = " << p.z;
  LOG(LTRACE) << "multiplicity: " << p.multiplicity;
  LOG(LTRACE) << "pointId " << p.pointId;
}

void NetworkBuilder::mapFeaturesNames()
{
  for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
    std::stringstream name;
    name << "F" << i;
    string featureName(name.str());
    features.insert(std::make_pair(i,featureName));
  }
}

}//: namespace Network
}//: namespace Processors
