/*!
 * \file CreateNetworkWithSpacialDependencies.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "CreateNetworkWithSpacialDependencies.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace pcl::octree;

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
	h_buildNetwork.setup(boost::bind(&CreateNetworkWithSpacialDependencies::buildNetwork, this));
	registerHandler("buildNetwork", &h_buildNetwork);
	addDependency("buildNetwork", &in_cloud_xyzsift);
//	addDependency("buildNetwork", &in_jointMultiplicity);

	registerStream("out_network", &out_network);
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

bool CreateNetworkWithSpacialDependencies::onStart()
{
    LOG(LTRACE) << "CreateNetworkWithSpacialDependencies::onStart\n";
    return true;
}

void CreateNetworkWithSpacialDependencies::buildNetwork() {
	LOG(LDEBUG) << "CreateNetworkWithSpacialDependencies::buildNetwork";

  if(network.getNumberOfNodes() != 0) {
    return;
  }

	// Read from dataport.
	cloud = in_cloud_xyzsift.read();
//  jointMultiplicityVector = in_jointMultiplicity.read();

	// Set voxel resolution.
	float voxelSize = 0.01f;
	OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId> octree (voxelSize);
	// Set input cloud.
	octree.setInputCloud(cloud);
	// Calculate bounding box of input cloud.
	octree.defineBoundingBox();

	// Add points from input cloud to octree.
	octree.addPointsFromInputCloud ();

//  addHypothesisNode();

  LOG(LDEBUG) << "Creating iterators";

	// Use depth-first iterator
  OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::DepthFirstIterator dfIt = octree.depth_begin();
	const OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::DepthFirstIterator dfIt_end = octree.depth_end();

  LOG(LDEBUG) << "Creating nodes";

	// Root node
	pcl::octree::OctreeNode* node = dfIt.getCurrentOctreeNode();
  OctreeBranchNode<OctreeContainerEmptyWithId>* parent;
  bool reachedLeafNode = false;

	if(node->getNodeType() == BRANCH_NODE) {
		OctreeBranchNode<OctreeContainerEmptyWithId>* rootNode = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>* > (node);
    createBranchNode(rootNode);
    parent = rootNode;
    addParentsToQueue(rootNode);
    ++dfIt;
	}

  for (;dfIt != dfIt_end; ++dfIt) {
    LOG(LDEBUG) << "========== Another node in depth search ==========";
    pcl::octree::OctreeNode* node = dfIt.getCurrentOctreeNode();
    if (node->getNodeType() == LEAF_NODE) {
      LOG(LDEBUG) << "Entering octree leaf node.";
      OctreeLeafNode< OctreeContainerPointIndicesWithId >* leafNode =   static_cast< OctreeLeafNode<OctreeContainerPointIndicesWithId>* > (node);
      createLeafNode(leafNode);
      connectLeafNode(leafNode, parent);
      createLeafNodeChildren(leafNode);
      reachedLeafNode = true;
    }
    else if (node->getNodeType() == BRANCH_NODE) {
      LOG(LDEBUG) << "Entering octree branch node.";
      if(reachedLeafNode) {
        parent = parentQueue.top();
        LOG(LDEBUG) << "Leaf node was reached in previous iteration. ";
        LOG(LDEBUG) << "Changing parent to: V_" << parent->getContainer().getNodeId();
        parentQueue.pop();
        reachedLeafNode = false;
      }
      OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>* > (node);
      if(nodeHasOnlyOneChild(branchNode)) {
        LOG(LDEBUG) << "Skipping octree node, that has only one child";
        continue;
      }
      else {
        LOG(LDEBUG) << "Node has multiple children, adding to Bayes network";
        addParentsToQueue(branchNode);
        createBranchNode(branchNode);
        connectBranchNode(branchNode, parent);
        parent = branchNode;
      }
    }
  }
  network.setCPTofAllVoxelNodes(numberOfVoxels);

	//	Delete octree data structure (pushes allocated nodes to memory pool!).
	octree.deleteTree ();

  exportNetwork();
}

void CreateNetworkWithSpacialDependencies::addParentsToQueue(OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode)
{
  LOG(LDEBUG) << "Adding parents to queue";
  if(!nodeHasOnlyOneChild(branchNode)) {
    int numberOfChildren = getNumberOfChildren(branchNode);
    for (int i=0; i<numberOfChildren-1; i++) {
      parentQueue.push(branchNode);
    }
    LOG(LDEBUG) << "Current node has " << numberOfChildren << " children, it appears in parent's queue " << numberOfChildren << " times";
  }
}

void CreateNetworkWithSpacialDependencies::createLeafNode(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode)
{
  //FIXME: Check for correctness and duplication. Is this method even necessary?
  LOG(LDEBUG) << "Creating leaf node: " << nextId;
  leafNode->getContainer().setNodeId(nextId);
  network.addVoxelNode(nextId);
  ++numberOfVoxels;
  ++nextId;
}

void CreateNetworkWithSpacialDependencies::connectLeafNode(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode, OctreeBranchNode<OctreeContainerEmptyWithId> *branchNode)
{
  int leafNodeId = leafNode->getContainer().getNodeId();
  string bayesParentNodeName = network.createVoxelName(leafNodeId);
  int parentId = branchNode->getContainer().getNodeId();
  string bayesChildNodeName = network.createVoxelName(parentId);
  network.addArc(bayesParentNodeName, bayesChildNodeName);
}

void CreateNetworkWithSpacialDependencies::createLeafNodeChildren(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode)
{
	LOG(LTRACE) << "----- Creating leaf node children -----";

	int parentId = leafNode->getContainer().getNodeId();
	int childrenCounter = leafNode->getContainer().getSize();

	// Iterate through container elements, i.e. cloud points.
	std::vector<int> point_indices;
	leafNode->getContainer().getPointIndices(point_indices);

	string parentName = network.createVoxelName(parentId);

  //FIXME: Change the way coefficients are calculated
  for(unsigned int i=0; i<leafNode->getContainer().getSize(); i++)
  {
    PointXYZSIFT p = cloud->at(point_indices[i]);
    logPoint(p, point_indices[i]);
    int featureId = p.pointId;
    network.addFeatureNode(featureId);
    string featureName = network.createFeatureName(featureId);
    network.addArc(featureName, parentName);
  }//: for points

	LOG(LTRACE) << "voxel ID: " << parentId;
	LOG(LTRACE) << "voxel name: " << network.createVoxelName(parentId);
	LOG(LTRACE) << "children count: " <<childrenCounter;
	leafNodeCount++;
}

bool CreateNetworkWithSpacialDependencies::nodeHasOnlyOneChild(OctreeBranchNode<OctreeContainerEmptyWithId> *branchNode)
{
  LOG(LTRACE) << "Check whether node has only one child";
  int childrenCounter = 0;
  for (unsigned child_idx = 0; child_idx < 8; ++child_idx) {
    if (branchNode -> hasChild(child_idx))
      ++childrenCounter;
  }
  LOG(LTRACE) << "Number of children: " << childrenCounter;
  if (childrenCounter == 1)
    return true;
  else
    return false;
}

bool CreateNetworkWithSpacialDependencies::nextNodeIsAlsoBranchNode(OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode)
{
  LOG(LTRACE) << "Check whether node's child is also a branch node";
  pcl::octree::OctreeNode* childNode;
  unsigned char index;
  for (index = 0; index < 8; ++index) {
    if (branchNode->hasChild(index))
			childNode = branchNode -> getChildPtr(index);
  }
  if (childNode->getNodeType() == BRANCH_NODE)
    return true;
  else
    return false;
}

int CreateNetworkWithSpacialDependencies::getNumberOfChildren(OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode)
{
  LOG(LTRACE) << "Get number of children from branch node";
  unsigned char index;
  int childrenCounter = 0;
  for (index = 0; index < 8; ++index) {
    if (branchNode->hasChild(index))
			++childrenCounter;
  }
  LOG(LTRACE) << "number of children: " << childrenCounter;
  return childrenCounter;
}

int CreateNetworkWithSpacialDependencies::getNumberOfChildren(OctreeLeafNode<OctreeContainerPointIndicesWithId>* leafNode)
{
  LOG(LTRACE) << "Get number of children from leaf node";
  LOG(LDEBUG) << "number of children: " << leafNode->getContainer().getSize();
  return leafNode->getContainer().getSize();
}

void CreateNetworkWithSpacialDependencies::createBranchNode(OctreeBranchNode<OctreeContainerEmptyWithId> *branchNode)
{
  //FIXME: duplication with createLeafNode method
  LOG(LDEBUG) << "Creating branch node: " << nextId;
  branchNode->getContainer().setNodeId(nextId);
  network.addVoxelNode(nextId);
  ++numberOfVoxels;
  ++nextId;
}

void CreateNetworkWithSpacialDependencies::connectBranchNode(OctreeBranchNode<OctreeContainerEmptyWithId> *branchNode, OctreeBranchNode<OctreeContainerEmptyWithId> *parentNode)
{
  int branchNodeId = branchNode->getContainer().getNodeId();
  string bayesParentNodeName = network.createVoxelName(branchNodeId);
  int parentId = parentNode->getContainer().getNodeId();
  string bayesChildNodeName = network.createVoxelName(parentId);
  network.addArc(bayesParentNodeName, bayesChildNodeName);
}

void CreateNetworkWithSpacialDependencies::exportNetwork()
{
	LOG(LTRACE) << "ELO! branchNodeCount: " << branchNodeCount;
	LOG(LTRACE) << "ELO! leafNodeCount: " << leafNodeCount;
	LOG(LTRACE) << "ELO! maxLeafContainerSize: " << maxLeafContainerSize;

	LOG(LDEBUG) << "before writing network to file";
  network.exportNetworkToFile();
	LOG(LDEBUG) << "after writing network to file";
	out_network.write(network.getNetwork());
}

void CreateNetworkWithSpacialDependencies::addHypothesisNode(int modelId)
{
  LOG(LDEBUG) << "Creating hypothesis node. Model id: " << modelId;
  stringstream name;
  name << "H_" << modelId;
  string hypothesisName = name.str();

  /*
   * FIXME: hardcoded modelId! Works only for one model.
   * For more models it'll probably cause conflicts with voxel nodes, which are enumerated with IDs > 0.
   * Possible solution: hypothesis node name may start with H_
   */
  network.addVoxelNode(modelId);
}

string CreateNetworkWithSpacialDependencies::getNodeName(int nodeHandle)
{
    return features[nodeHandle];
}

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
