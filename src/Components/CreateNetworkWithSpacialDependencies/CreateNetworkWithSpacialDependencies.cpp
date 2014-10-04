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
  
	if(theNet.GetNumberOfNodes() != 0) {
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

	// Use breadth-first iterator
	OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt = octree.breadth_begin();
	const OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt_end = octree.breadth_end();

	// Use depth-first iterator
  OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::DepthFirstIterator dfIt = octree.depth_begin();
	const OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::DepthFirstIterator dfIt_end = octree.depth_end();

  LOG(LDEBUG) << "Creating nodes";
  
//  int nextId = 0;
	// Root node
	pcl::octree::OctreeNode* node = dfIt.getCurrentOctreeNode(); 
  OctreeBranchNode<OctreeContainerEmptyWithId>* parent;
  OctreeBranchNode<OctreeContainerEmptyWithId>* previousParent;
  bool reachedLeafNode = false;
  
	if(node->getNodeType() == BRANCH_NODE) {
		OctreeBranchNode<OctreeContainerEmptyWithId>* rootNode = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>* > (node);
//		LOG(LDEBUG) << "branch node: " << rootNode->getContainer().getNodeId();
    createBranchNode(rootNode);
    parent = rootNode;
    previousParent = parent;
    nodeHasOnlyOneChild(rootNode);
//		LOG(LDEBUG) << "root id: " << rootNode->getContainer().getNodeId();
//    LOG(LDEBUG) << "parent pointer: " << parent;
    ++dfIt;
	}
  
  for (;dfIt != dfIt_end; ++dfIt) {
    LOG(LDEBUG) << "----- Another node in depth search -----";
//    LOG(LDEBUG) << "current parent: " << parent->getContainer().getNodeId();
//    LOG(LDEBUG) << "parent pointer: " << parent;
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
//        LOG(LDEBUG) << "Reached leaf node.";
        parent = previousParent;
        reachedLeafNode = false;
      }
//			LOG(LDEBUG) << "current parent: " << parent->getContainer().getNodeId();
//			LOG(LDEBUG) << "parent pointer: " << parent;
			OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>* > (node);
//      if(nodeHasOnlyOneChild(branchNode)) {
//        LOG(LDEBUG) << "Skipping octree node, that has only one child";
//        continue;
//      }
//      else {
        LOG(LDEBUG) << "Node has multiple children, adding to Bayes network";
//				LOG(LDEBUG) << "current parent: " << parent->getContainer().getNodeId();
//        LOG(LDEBUG) << "branch node: " << branchNode->getContainer().getNodeId();
        createBranchNode(branchNode);
//				LOG(LDEBUG) << "parent pointer: " << parent;
//				LOG(LDEBUG) << "node pointer: " << branchNode;
//        LOG(LDEBUG) << "parent: " << parent->getContainer().getNodeId();
//        LOG(LDEBUG) << "child: " << branchNode->getContainer().getNodeId();
        connectBranchNode(branchNode, parent);
        parent = branchNode;
        if(nextNodeIsAlsoBranchNode(branchNode) && !nodeHasOnlyOneChild(branchNode)) {
					previousParent = parent;
        }
//      }
    }
  }
//  setCPTofAllNodes();

	//	Delete octree data structure (pushes allocated nodes to memory pool!).
	octree.deleteTree ();
  
  exportNetwork();
}

void CreateNetworkWithSpacialDependencies::createLeafNode(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode)
{
  //FIXME: Check for correctness and duplication. Is this method even necessary? 
//  LOG(LDEBUG) << "Creating leaf node: " << nodeId;
//  addVoxelNode(nodeId);
  LOG(LDEBUG) << "Creating leaf node: " << nextId;
  leafNode->getContainer().setNodeId(nextId);
  addVoxelNode(nextId);
//  string nodeName = createVoxelName(nextId);
//  int numberOfChildren = getNumberOfChildren(leafNode);
//  setNodeCPT(nodeName, numberOfChildren);
  ++nextId;
}

void CreateNetworkWithSpacialDependencies::connectLeafNode(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode, OctreeBranchNode<OctreeContainerEmptyWithId> *branchNode)
{
  //TODO: FIXME: evaluate for proper order
  int leafNodeId = leafNode->getContainer().getNodeId();
  string bayesParentNodeName = createVoxelName(leafNodeId);
  int parentId = branchNode->getContainer().getNodeId();
  string bayesChildNodeName = createVoxelName(parentId);
  LOG(LTRACE) << "Connecting nodes: " << bayesParentNodeName << "->" << bayesChildNodeName;
  addArc(bayesParentNodeName, bayesChildNodeName);
}

void CreateNetworkWithSpacialDependencies::createLeafNodeChildren(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leafNode)
{
	LOG(LTRACE) << "Creating leaf node children";

	int parentId = leafNode->getContainer().getNodeId();
	int childrenCounter = leafNode->getContainer().getSize();

	// Iterate through container elements, i.e. cloud points.
	std::vector<int> point_indices;
	leafNode->getContainer().getPointIndices(point_indices);

	string parentName = createVoxelName(parentId);

	//FIXME: Change the way coefficients are calculated
	for(unsigned int i=0; i<leafNode->getContainer().getSize(); i++)
	{
		PointXYZSIFT p = cloud->at(point_indices[i]);
		logPoint(p, point_indices[i]);
		string featureName = createFeatureName(p.pointId);
    LOG(LDEBUG) << "Adding new feature: " << featureName;
		addNode(featureName);
		addArc(featureName, parentName);
	}//: for points		

	LOG(LDEBUG) << "voxel ID: " << parentId;
	LOG(LDEBUG) << "voxel name: " << createVoxelName(parentId);
	LOG(LDEBUG) << "children count: " <<childrenCounter;
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
  LOG(LDEBUG) << "Number of children: " << childrenCounter;
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
//  LOG(LDEBUG) << "Child node: " << childNode;
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
  LOG(LDEBUG) << "number of children: " << childrenCounter;
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
//  int nodeId = branchNode -> getContainer().getNodeId();
//  LOG(LDEBUG) << "Creating branch node: " << nodeId;
  LOG(LDEBUG) << "Creating branch node: " << nextId;
  branchNode->getContainer().setNodeId(nextId);
  addVoxelNode(nextId);
//  string nodeName = createVoxelName(nextId);
//  int numberOfChildren = getNumberOfChildren(branchNode);
//  setNodeCPT(nodeName, numberOfChildren);
  ++nextId;
}

void CreateNetworkWithSpacialDependencies::connectBranchNode(OctreeBranchNode<OctreeContainerEmptyWithId> *branchNode, OctreeBranchNode<OctreeContainerEmptyWithId> *parentNode)
{
  //FIXME: check out FIXME in connectLeafNode method
  int branchNodeId = branchNode->getContainer().getNodeId();
  string bayesParentNodeName = createVoxelName(branchNodeId);
  int parentId = parentNode->getContainer().getNodeId();
  string bayesChildNodeName = createVoxelName(parentId);
  LOG(LTRACE) << "Connecting nodes: " << bayesParentNodeName << "->" << bayesChildNodeName;
  addArc(bayesParentNodeName, bayesChildNodeName);
}

void CreateNetworkWithSpacialDependencies::exportNetwork()
{
	LOG(LWARNING) << "ELO! branchNodeCount: " << branchNodeCount;
	LOG(LWARNING) << "ELO! leafNodeCount: " << leafNodeCount;
	LOG(LWARNING) << "ELO! maxLeafContainerSize: " << maxLeafContainerSize;

	LOG(LDEBUG) << "before writing network to file";
	theNet.WriteFile("out_network.xdsl", DSL_XDSL_FORMAT);
	LOG(LDEBUG) << "after writing network to file";
	out_network.write(theNet);
}

void CreateNetworkWithSpacialDependencies::addHypothesisNode() 
{
	int modelId = 0;
  
  /*
   * FIXME: hardcoded modelId! Works only for one model. 
   * For more models it'll probably cause conflicts with voxel nodes, which are enumerated with IDs > 0.
   * Possible solution: hypothesis node name may start with H_
   */
  string hypothesisName = createVoxelName(modelId);
	addNode(hypothesisName);
}


void CreateNetworkWithSpacialDependencies::createChild(pcl::octree::OctreeNode* child, int parentId)
{
	if(child->getNodeType() == BRANCH_NODE) {
		OctreeBranchNode<OctreeContainerEmptyWithId>* child_node = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>*> (child);
		child_node->getContainer().setNodeId(nextId++);
		LOG(LDEBUG) << "node id: " << child_node->getContainer().getNodeId();
	}
	else if(child->getNodeType() == LEAF_NODE) {
		OctreeLeafNode<OctreeContainerPointIndicesWithId>* child_node = static_cast<OctreeLeafNode<OctreeContainerPointIndicesWithId>* >(child);
		child_node->getContainer().setNodeId(nextId++);
		LOG(LDEBUG) << "node id: " << child_node->getContainer().getNodeId();
	}
  else {
    return;
  }
	int currentId = nextId - 1;
	addVoxelNode(currentId);
	addArc(createVoxelName(currentId), createVoxelName(parentId));
}

void CreateNetworkWithSpacialDependencies::addVoxelNode(int id)
{
	string voxelName = createVoxelName(id);
  addNode(voxelName);
  ++numberOfVoxels;
}

string CreateNetworkWithSpacialDependencies::createVoxelName(int id)
{
	stringstream name;
	name << "V_" << id;
	string voxelName = name.str();
  return voxelName;
}

string CreateNetworkWithSpacialDependencies::createFeatureName(int id)
{
	stringstream name;
	name << "F_" << id;
	string featureName(name.str());
	return featureName;
}

void CreateNetworkWithSpacialDependencies::addNode(std::string name)
{
    LOG(LDEBUG) << "Add node to network: " << name;
    int newNode = theNet.AddNode(DSL_CPT, name.c_str());
    DSL_idArray outcomes;
    std::vector<string> outcomesNames;
    outcomesNames.push_back("YES");
    outcomesNames.push_back("NO");
    for (int i=0; i<outcomesNames.size(); i++) {
        outcomes.Add(outcomesNames[i].c_str());
    }
    theNet.GetNode(newNode)->Definition()->SetNumberOfOutcomes(outcomes);
}

void CreateNetworkWithSpacialDependencies::setCPTofAllNodes()
{
  LOG(LTRACE) << "Setting CPT of all nodes";

  for (unsigned int i=0; i<numberOfVoxels; ++i) {
    LOG(LTRACE) << "Setting CPT of voxel number " << i << " " << createVoxelName(i);
    int numberOfChildren = theNet.NumParents(i);
    LOG(LTRACE) << "Number of ancestors: " << numberOfChildren;
    if(numberOfChildren == 0) 
      continue;
    string nodeName = createVoxelName(i);
    setNodeCPT(nodeName, numberOfChildren);
  }
}

void CreateNetworkWithSpacialDependencies::setNodeCPT(string name, int numberOfParents)
{
	LOG(LDEBUG) << "Set node CPT: " << name;
	std::vector<double> probabilities;
	std::string s(numberOfParents,'1');
	do {
		int ones = std::count(s.begin(),s.end(),'1');
		double probability = (double) ones/numberOfParents;
		probabilities.push_back(probability);
		probabilities.push_back(1-probability);
	} while(generateNext(s.begin(), s.end()));

	fillCPT(name, probabilities);
}

void CreateNetworkWithSpacialDependencies::addArc(string parentName, string childName)
{
	int childNode = theNet.FindNode(childName.c_str());
	int parentNode = theNet.FindNode(parentName.c_str());
	theNet.AddArc(parentNode, childNode);
}

void CreateNetworkWithSpacialDependencies::fillCPT(string name, std::vector<double> probabilities)
{
	int node = theNet.FindNode(name.c_str());
	DSL_sysCoordinates theCoordinates(*theNet.GetNode(node)->Definition());

	std::vector<double>::iterator it = probabilities.begin();
	do {
		theCoordinates.UncheckedValue() = *it;
		LOG(LTRACE) << "Probability: " << *it;
		++it;
	} while(theCoordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

int CreateNetworkWithSpacialDependencies::generateNext(std::string::iterator start, std::string::iterator end)
{
	while(start != end)
	{
		--end;
		if ((*end & 1) == 1)
		{
			--*end;
			return true;
		}
		else
		{
			++*end;
		}
	}
	return false;
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
