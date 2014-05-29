/*!
 * \file CreateNetwork.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "CreateNetwork.hpp"
#include "OctreeContainers.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace pcl::octree;

namespace Processors {
namespace Network {

CreateNetwork::CreateNetwork(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello CreateNetwork\n";
    branchNodeCount = 0;
    leafNodeCount = 0;
    maxLeafContainerSize = 0;
    nextId = 0;
}

CreateNetwork::~CreateNetwork()
{
    LOG(LTRACE)<<"Good bye CreateNetwork\n";
}

void CreateNetwork::prepareInterface()
{
	LOG(LTRACE) << "CreateNetwork::prepareInterface\n";

	// Register data streams.
	//	registerStream("in_cloud", &in_cloud_xyz);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_jointMultiplicity", &in_jointMultiplicity);
	// Register handlers
	h_buildNetwork.setup(boost::bind(&CreateNetwork::buildNetwork, this));
	registerHandler("buildNetwork", &h_buildNetwork);
	addDependency("buildNetwork", &in_cloud_xyzsift);
	addDependency("buildNetwork", &in_jointMultiplicity);

	registerStream("out_network", &out_network);
}

bool CreateNetwork::onInit()
{
    LOG(LTRACE) << "CreateNetwork::initialize\n";
    return true;
}

bool CreateNetwork::onFinish()
{
    LOG(LTRACE) << "CreateNetwork::finish\n";
    return true;
}

bool CreateNetwork::onStop()
{
    LOG(LTRACE) << "CreateNetwork::onStop\n";
    return true;
}

bool CreateNetwork::onStart()
{
    LOG(LTRACE) << "CreateNetwork::onStart\n";
    return true;
}

void CreateNetwork::buildNetwork() {
	LOG(LDEBUG) << "CreateNetwork::buildNetwork";
  
	if(theNet.GetNumberOfNodes() != 0) {
		return;
	}

	// Read from dataport.
	cloud = in_cloud_xyzsift.read();
  jointMultiplicityVector = in_jointMultiplicity.read();

	// Set voxel resolution.
	float voxelSize = 0.01f;
	OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId> octree (voxelSize);
	// Set input cloud.
	octree.setInputCloud(cloud);
	// Calculate bounding box of input cloud.
	octree.defineBoundingBox();

	// Add points from input cloud to octree.
	octree.addPointsFromInputCloud ();

  addHypothesisNode();

	// Use breadth-first iterator
	OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt = octree.breadth_begin();
	const OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt_end = octree.breadth_end();

	// Root node
	pcl::octree::OctreeNode* node = bfIt.getCurrentOctreeNode(); 
	if(node->getNodeType() == BRANCH_NODE) {
		OctreeBranchNode<OctreeContainerEmptyWithId>* branch_node = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>* > (node);
		branch_node->getContainer().setNodeId(nextId);
		nextId++;
		LOG(LINFO) << "root id: " << branch_node->getContainer().getNodeId();
	}

	for (; bfIt != bfIt_end; ++bfIt)
	{
		LOG(LINFO) << "depth = " << bfIt.getCurrentOctreeDepth ();
		pcl::octree::OctreeNode* node = bfIt.getCurrentOctreeNode(); 
    
		if (node->getNodeType () == BRANCH_NODE) {
			LOG(LDEBUG) << "BRANCH";
      createBranchNodeChildren(node);
		}
    if (node->getNodeType () == LEAF_NODE) {
			LOG(LINFO) << "LEAF";
      createLeafNodeChildren(node);
		}
	}

	//	Delete octree data structure (pushes allocated nodes to memory pool!).
	octree.deleteTree ();
  
  exportNetwork();
}

void CreateNetwork::exportNetwork()
{
	LOG(LWARNING) << "ELO! branchNodeCount: " << branchNodeCount;
	LOG(LWARNING) << "ELO! leafNodeCount: " << leafNodeCount;
	LOG(LWARNING) << "ELO! maxLeafContainerSize: " << maxLeafContainerSize;

	LOG(LDEBUG) << "before writing network to file";
	theNet.WriteFile("out_network.xdsl", DSL_XDSL_FORMAT);
	LOG(LDEBUG) << "after writing network to file";
	out_network.write(theNet);
}

void CreateNetwork::addHypothesisNode() 
{
	int modelId = 0;
//	int summedFeaturesMultiplicity = 0;
	//    for(unsigned i=0; i<cloud->size(); ++i) {
	//        summedFeaturesMultiplicity += cloud->at(i).multiplicity;
	//    }
  
  /*
   * FIXME: hardcoded modelId! Works only for one model. 
   * For more models it'll probably cause conflicts with voxel nodes, which are enumerated with IDs > 0.
   * Possible solution: hypothesis node name may start with H_
   */
  string hypothesisName = createVoxelName(modelId);
	addNode(hypothesisName);
}

//TODO: refactor
void CreateNetwork::createBranchNodeChildren(pcl::octree::OctreeNode* node) 
{
	OctreeBranchNode<OctreeContainerEmptyWithId>* branch_node = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>* > (node);
	int parentId = branch_node->getContainer().getNodeId();
	LOG(LDEBUG) << "root id: " << parentId;

	// iterate over all children
	unsigned char child_idx;
	int childrenCounter = 0;
	for (child_idx = 0; child_idx < 8 ; ++child_idx) {
		if (branch_node->hasChild(child_idx)) {
			LOG(LDEBUG) << "Child "<<(int)child_idx << "present";
			childrenCounter++;
			OctreeNode* child = branch_node->getChildPtr(child_idx);
      createChild(child, parentId);
		}//:if has child
	}//:for children
  
	string voxelName = createVoxelName(parentId);
	LOG(LDEBUG) << "voxel ID: " << parentId;
	LOG(LDEBUG) << "voxel name: " << voxelName;
	LOG(LDEBUG) << "children count: " <<childrenCounter;
	setNodeCPT(voxelName, childrenCounter);
	branchNodeCount++;
}

//TODO: refactor
void CreateNetwork::createLeafNodeChildren(pcl::octree::OctreeNode* node)
{
	OctreeLeafNode< OctreeContainerPointIndicesWithId >* leaf_node =   static_cast< OctreeLeafNode<OctreeContainerPointIndicesWithId>* > (node);
	int containter_size = leaf_node->getContainer().getSize();
	if(containter_size >8) {
		LOG(LWARNING) << "Leaf containter has big number of features! (" << containter_size << ")";
	}//: if
	if(containter_size > maxLeafContainerSize)
		maxLeafContainerSize = containter_size;

	int parentId = leaf_node->getContainer().getNodeId();
	int childrenCounter = leaf_node->getContainer().getSize();
//	double p[] = {0,1.0};
//	std::vector<double> featureInitialProbabilities (p, p + sizeof(p) / sizeof(double) );
	std::vector<double> featuresCoefficients;
	int summedFeaturesMultiplicity = 0;

	// Iterate through container elements, i.e. cloud points.
	std::vector<int> point_indices;
	leaf_node->getContainer().getPointIndices(point_indices);
	for(unsigned int j=0; j<leaf_node->getContainer().getSize(); j++) {
		PointXYZSIFT p = cloud->at(point_indices[j]);
		summedFeaturesMultiplicity += p.multiplicity;
	}
	for(unsigned int i=0; i<leaf_node->getContainer().getSize(); i++)
	{
		LOG(LTRACE) << "Iteration number " << i << " Point index=" << point_indices[i];
		PointXYZSIFT p = cloud->at(point_indices[i]);
		LOG(LTRACE) << "p.x = " << p.x << " p.y = " << p.y << " p.z = " << p.z;
		LOG(LTRACE) << "multiplicity: " << p.multiplicity;
		LOG(LTRACE) << "pointId " << p.pointId;
    
    string featureName = createFeatureName(p.pointId);
    string parentName = createVoxelName(parentId);
		addNode(featureName);
		addArc(featureName, parentName);
//    fillCPT(featureName, featureInitialProbabilities);
		double coefficient = (double) p.multiplicity/(jointMultiplicityVector[i] * childrenCounter * 2);
		featuresCoefficients.push_back(coefficient);
	}//: for points		
  
	LOG(LDEBUG) << "voxel ID: " << parentId;
	LOG(LDEBUG) << "voxel name: " << createVoxelName(parentId);
	LOG(LDEBUG) << "children count: " <<childrenCounter;
  setVoxelNodeCPT(parentId, featuresCoefficients, childrenCounter);
	leafNodeCount++;
}

void CreateNetwork::createChild(pcl::octree::OctreeNode* child, int parentId)
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

void CreateNetwork::addVoxelNode(int id)
{
	stringstream name;
	name << "V_" << id;
	string voxelName = name.str();
  addNode(voxelName);
}

string CreateNetwork::createVoxelName(int id)
{
	stringstream name;
	name << "V_" << id;
	string voxelName = name.str();
  return voxelName;
}

string CreateNetwork::createFeatureName(int id)
{
	stringstream name;
	name << "F_" << id;
	string featureName(name.str());
	return featureName;
}

//TODO: make functionality more general
void CreateNetwork::setVoxelNodeCPT(int id, std::vector<double> featuresCoefficients, int childrenCounter) 
{
  string voxelName = createVoxelName(id);
	setNodeCPT(voxelName, featuresCoefficients);
	//            setNodeCPT(voxelName, childrenCounter);
}

void CreateNetwork::addNode(std::string name)
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

void CreateNetwork::addArc(string parentName, string childName)
{
	int childNode = theNet.FindNode(childName.c_str());
	int parentNode = theNet.FindNode(parentName.c_str());
	theNet.AddArc(parentNode, childNode);
}

//TODO: split into two methods: calculating probability and actually setting node's cpt
void CreateNetwork::setNodeCPT(string name, int numberOfParents)
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

void CreateNetwork::setNodeCPT(string name, std::vector<double> parentsCoefficients)
{
	LOG(LDEBUG) << "Set node CPT: " << name;
	std::vector<double> probabilities;
	std::string s(parentsCoefficients.size(),'1');
	do {
		int ones = std::count(s.begin(),s.end(),'1');
		double probability = 0; //(double) ones/numberOfParents;
		for(unsigned i=0; i<parentsCoefficients.size(); ++i) {
			if(s.at(i) == '1') {
				probability += parentsCoefficients[i];
			}
		}
		probability = probability/parentsCoefficients.size();
		probabilities.push_back(probability);
		probabilities.push_back(1-probability);
	} while(generateNext(s.begin(), s.end()));
  
	fillCPT(name, probabilities);
}

void CreateNetwork::fillCPT(string name, std::vector<double> probabilities)
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

int CreateNetwork::generateNext(std::string::iterator start, std::string::iterator end)
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

string CreateNetwork::getNodeName(int nodeHandle)
{
    return features[nodeHandle];
}

void CreateNetwork::loadNetwork()
{
    int result = -1;
    //result = theNet.ReadFile("/home/qiubix/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    //result = theNet.ReadFile("/home/kkaterza/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    LOG(LWARNING) << "Reading network file: " << result;
}

void CreateNetwork::mapFeaturesNames()
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
