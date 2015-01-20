/*!
 * \file CreateNetworkWithSpacialDependencies.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <assert.h>

#include "CreateNetworkWithSpacialDependencies.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

CreateNetworkWithSpacialDependencies::CreateNetworkWithSpacialDependencies(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello CreateNetworkWithSpacialDependencies\n";
  branchNodeCount = 0;
  leafNodeCount = 0;
  maxLeafContainerSize = 0;
  nextId = 0;
  numberOfVoxels = 0;
  featureNodeCount = 0;
}

CreateNetworkWithSpacialDependencies::~CreateNetworkWithSpacialDependencies()
{
    LOG(LTRACE)<<"Good bye CreateNetworkWithSpacialDependencies\n";
}

void CreateNetworkWithSpacialDependencies::prepareInterface()
{
	LOG(LTRACE) << "CreateNetworkWithSpacialDependencies::prepareInterface\n";

	// Register data streams.
	//	registerStream("in_cloud", &in_cloud_xyz);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_jointMultiplicity", &in_jointMultiplicity);
	// Register handlers
	h_onNewModel.setup(boost::bind(&CreateNetworkWithSpacialDependencies::onNewModel, this));
	h_onJointMultiplicity.setup(boost::bind(&CreateNetworkWithSpacialDependencies::onJointMultiplicity, this));
	registerHandler("onNewModel", &h_onNewModel);
	registerHandler("onJointMultiplicity", &h_onJointMultiplicity);
	addDependency("onNewModel", &in_cloud_xyzsift);
	addDependency("onJointMultiplicity", &in_jointMultiplicity);

	//registerStream("out_network", &out_network);
	registerStream("out_networks", &out_networks);
}

bool CreateNetworkWithSpacialDependencies::onInit()
{
    LOG(LTRACE) << "CreateNetworkWithSpacialDependencies::initialize\n";
    return true;
}

bool CreateNetworkWithSpacialDependencies::onFinish()
{
    LOG(LTRACE) << "CreateNetworkWithSpacialDependencies::finish\n";
    return true;
}

bool CreateNetworkWithSpacialDependencies::onStop()
{
    LOG(LTRACE) << "CreateNetworkWithSpacialDependencies::onStop\n";
    return true;
}

void CreateNetworkWithSpacialDependencies::onNewModel()
{
  LOG(LTRACE) << "On new model";
  pcl::PointCloud<PointXYZSIFT>::Ptr newCloud = in_cloud_xyzsift.read();
  cloudQueue.push(newCloud);
}

void CreateNetworkWithSpacialDependencies::onJointMultiplicity()
{
  LOG(LTRACE) << "On joint multiplicity";
  jointMultiplicityVector = in_jointMultiplicity.read();
  if (cloudQueue.size() > 0) {
    buildNetwork();
  }
}

bool CreateNetworkWithSpacialDependencies::onStart()
{
    LOG(LTRACE) << "CreateNetworkWithSpacialDependencies::onStart\n";
    return true;
}

void CreateNetworkWithSpacialDependencies::buildNetwork() {
  LOG(LDEBUG) << " #################### Building network ################### ";

  if(network.getNumberOfNodes() != 0) {
    return;
  }

  LOG(LDEBUG) << "Size of cloudQueue: " << cloudQueue.size();
  // Read from queue
  cloud = cloudQueue.top();
  cloudQueue.pop();
  //  jointMultiplicityVector = in_jointMultiplicity.read();

  Octree octree(cloud);
  octree.init();

  LOG(LTRACE) << "Creating iterators...";

  // Use depth-first iterator
  Octree::DepthFirstIterator dfIt = octree.depthBegin();
  const Octree::DepthFirstIterator dfItEnd = octree.depthEnd();

  LOG(LTRACE) << "Creating nodes:";

  OctreeNode node = *dfIt;

  // Root node
  if(node.getNodeType() == OCTREE_BRANCH_NODE) {
    OctreeBranchNode root(node);
    addHypothesisNode(root);
    ++dfIt;
  }
  else {
    LOG(LDEBUG) << "Error creating hypothesis node. First node is not a branch node!";
    return;
  }

  for (;dfIt != dfItEnd; ++dfIt) {
    LOG(LDEBUG) << "========== Another node in depth search ==========";
    OctreeNode node = *dfIt;
    if (node.getNodeType() == OCTREE_LEAF_NODE) {
      LOG(LDEBUG) << "Entering octree leaf node.";
      OctreeLeafNode leafNode(node);
      createLeafNode(leafNode);
      connectNodeToNetwork(&leafNode);
      createLeafNodeChildren(leafNode);
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
        createBranchNode(branchNode);
        connectNodeToNetwork(&branchNode);
        addParentsToQueue(branchNode);
        ++branchNodeCount;
      }
    }
  }
  network.setCPTofAllVoxelNodes(numberOfVoxels);

  exportNetwork();
}

void CreateNetworkWithSpacialDependencies::addParentsToQueue(OctreeBranchNode branchNode)
{
  LOG(LDEBUG) << "Adding parents to queue";
  if(!branchNode.hasOnlyOneChild()) {
    int numberOfChildren = branchNode.getNumberOfChildren();
    for (int i=0; i<numberOfChildren; i++) {
      parentQueue.push(branchNode);
    }
    LOG(LDEBUG) << "Current node has " << numberOfChildren << " children, it appears in parent's queue " << numberOfChildren << " times";
    LOG(LTRACE) << "Parent queue size: " << parentQueue.size();
  }
}

void CreateNetworkWithSpacialDependencies::createLeafNode(OctreeLeafNode leafNode)
{
  //FIXME: Check for correctness and duplication. Is this method even necessary?
  LOG(LDEBUG) << "Creating leaf node: " << nextId;
  leafNode.setId(nextId);
  network.addVoxelNode(nextId);
  ++numberOfVoxels;
  ++nextId;
}

void CreateNetworkWithSpacialDependencies::createLeafNodeChildren(OctreeLeafNode leafNode)
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
    network.addArc(featureName, parentName);
    ++featureNodeCount;
  }//: for points

  LOG(LTRACE) << "voxel ID: " << parentId;
  LOG(LTRACE) << "voxel name: " << network.createVoxelName(parentId);
  LOG(LTRACE) << "children count: " <<childrenCounter;
  leafNodeCount++;
}

void CreateNetworkWithSpacialDependencies::createBranchNode(OctreeBranchNode branchNode)
{
  //FIXME: duplication with createLeafNode method
  LOG(LDEBUG) << "Creating branch node: " << nextId;
  branchNode.setId(nextId);
  network.addVoxelNode(nextId);
  ++numberOfVoxels;
  ++nextId;
}

void CreateNetworkWithSpacialDependencies::connectNodeToNetwork(OctreeNode* child)
{
  int childId = child->getId();
  string bayesParentNodeName = network.createVoxelName(childId);
  OctreeBranchNode parent(parentQueue.top());
  int parentId = parent.getId();
  string bayesChildNodeName = network.createVoxelName(parentId);
  parentQueue.pop();
  network.addArc(bayesParentNodeName, bayesChildNodeName);
}

void CreateNetworkWithSpacialDependencies::exportNetwork()
{
	LOG(LWARNING) << "ELO! Branch node quantity: " << branchNodeCount;
	LOG(LWARNING) << "ELO! Leaf node quantity: " << leafNodeCount;
	LOG(LWARNING) << "ELO! Voxel node quantity: " << numberOfVoxels;
	LOG(LWARNING) << "ELO! Feature node quantity: " << featureNodeCount;
	LOG(LWARNING) << "ELO! maxLeafContainerSize: " << maxLeafContainerSize;

	LOG(LDEBUG) << "before writing network to file";
  network.exportNetworkToFile();
	LOG(LDEBUG) << "after writing network to file";
  std::vector<DSL_network> networks;
  networks.push_back(network.getNetwork());
  out_networks.write(networks);
	//out_network.write(network.getNetwork());
}

void CreateNetworkWithSpacialDependencies::addHypothesisNode(OctreeBranchNode root, int modelId)
{
  LOG(LDEBUG) << "Creating hypothesis node. Model id: " << modelId;
  /*
   * FIXME: hardcoded modelId! Works only for one model.
   * For more models it'll probably cause conflicts with voxel nodes, which are enumerated with IDs > 0.
   * Possible solution: hypothesis node name may start with H_
   */
  createBranchNode(root);
  addParentsToQueue(root);
  ++branchNodeCount;
}

string CreateNetworkWithSpacialDependencies::getNodeName(int nodeHandle)
{
    return features[nodeHandle];
}

/*
void CreateNetworkWithSpacialDependencies::logLeafNodeContainerSize(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode)
{
	int containter_size = leafNode->getContainer().getSize();
	if(containter_size >8) {
		LOG(LWARNING) << "Leaf containter has big number of features! (" << containter_size << ")";
	}//: if
	if(containter_size > maxLeafContainerSize)
    maxLeafContainerSize = containter_size;
}

int CreateNetworkWithSpacialDependencies::sumMultiplicityInsideVoxel(pcl::octree::OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode)
{
  int summedFeaturesMultiplicity = 0;
	std::vector<int> point_indices;
	leafNode->getContainer().getPointIndices(point_indices);
	for(unsigned int j=0; j<leafNode->getContainer().getSize(); j++) {
		PointXYZSIFT p = cloud->at(point_indices[j]);
		summedFeaturesMultiplicity += p.multiplicity;
	}
  return summedFeaturesMultiplicity;
}
*/

void CreateNetworkWithSpacialDependencies::logPoint(PointXYZSIFT p, int index)
{
		LOG(LTRACE) << "Point index = " << index;
		LOG(LTRACE) << "p.x = " << p.x << " p.y = " << p.y << " p.z = " << p.z;
		LOG(LTRACE) << "multiplicity: " << p.multiplicity;
		LOG(LTRACE) << "pointId " << p.pointId;
}

void CreateNetworkWithSpacialDependencies::mapFeaturesNames()
{
    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
        std::stringstream name;
        name << "F" << i;
        string featureName(name.str());
        features.insert(std::make_pair<int,string>(i,featureName));
    }
}

}//: namespace Network
}//: namespace Processors
