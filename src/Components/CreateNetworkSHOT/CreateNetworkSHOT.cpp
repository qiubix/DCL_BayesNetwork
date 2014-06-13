/*!
 * \file CreateNetworkSHOT.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "CreateNetworkSHOT.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace pcl::octree;

namespace Processors {
namespace Network {

CreateNetworkSHOT::CreateNetworkSHOT(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello CreateNetworkSHOT\n";
    branchNodeCount = 0;
    leafNodeCount = 0;
    maxLeafContainerSize = 0;
    nextId = 0;
}

CreateNetworkSHOT::~CreateNetworkSHOT()
{
    LOG(LTRACE)<<"Good bye CreateNetworkSHOT\n";
}

void CreateNetworkSHOT::prepareInterface()
{
	LOG(LTRACE) << "CreateNetworkSHOT::prepareInterface\n";

	// Register data streams.
	//	registerStream("in_cloud", &in_cloud_xyz);
//	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
//	registerStream("in_jointMultiplicity", &in_jointMultiplicity);
	registerStream("in_models", &in_models);
	// Register handlers
	h_buildNetwork.setup(boost::bind(&CreateNetworkSHOT::buildNetwork, this));
	registerHandler("buildNetwork", &h_buildNetwork);
	addDependency("buildNetwork", &in_models);
//	addDependency("buildNetwork", &in_jointMultiplicity);

	registerStream("out_network", &out_network);
}

bool CreateNetworkSHOT::onInit()
{
    LOG(LTRACE) << "CreateNetworkSHOT::initialize\n";
    return true;
}

bool CreateNetworkSHOT::onFinish()
{
    LOG(LTRACE) << "CreateNetworkSHOT::finish\n";
    return true;
}

bool CreateNetworkSHOT::onStop()
{
    LOG(LTRACE) << "CreateNetworkSHOT::onStop\n";
    return true;
}

bool CreateNetworkSHOT::onStart()
{
    LOG(LTRACE) << "CreateNetworkSHOT::onStart\n";
    return true;
}

void CreateNetworkSHOT::buildNetwork() {
	LOG(LDEBUG) << "CreateNetworkSHOT::buildNetwork";
  
	if(theNet.GetNumberOfNodes() != 0) {
		return;
	}

	// Read from dataport.
//	cloud = in_cloud_xyzshot.read();
//  jointMultiplicityVector = in_jointMultiplicity.read();
  std::vector<AbstractObject*> models = in_models.read();
  S2ObjectModel* model = dynamic_cast<S2ObjectModel*>(models[0]);
  cloud = model->cloud_xyzshot;

	for(unsigned i=0; i<cloud->size(); i++) {
		cloud->at(i).pointId = i;
	}
    
	// Set voxel resolution.
	float voxelSize = 0.01f;
	OctreePointCloud<PointXYZSHOT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId> octree (voxelSize);
	// Set input cloud.
	octree.setInputCloud(cloud);
	// Calculate bounding box of input cloud.
	octree.defineBoundingBox();

	// Add points from input cloud to octree.
	octree.addPointsFromInputCloud ();

  addHypothesisNode();

	// Use breadth-first iterator
	OctreePointCloud<PointXYZSHOT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt = octree.breadth_begin();
	const OctreePointCloud<PointXYZSHOT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt_end = octree.breadth_end();

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

void CreateNetworkSHOT::exportNetwork()
{
	LOG(LWARNING) << "ELO! branchNodeCount: " << branchNodeCount;
	LOG(LWARNING) << "ELO! leafNodeCount: " << leafNodeCount;
	LOG(LWARNING) << "ELO! maxLeafContainerSize: " << maxLeafContainerSize;

	LOG(LDEBUG) << "before writing network to file";
	theNet.WriteFile("out_network.xdsl", DSL_XDSL_FORMAT);
	LOG(LDEBUG) << "after writing network to file";
	out_network.write(theNet);
}

void CreateNetworkSHOT::addHypothesisNode() 
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
void CreateNetworkSHOT::createBranchNodeChildren(pcl::octree::OctreeNode* node) 
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

void CreateNetworkSHOT::createLeafNodeChildren(pcl::octree::OctreeNode* node)
{
	OctreeLeafNode< OctreeContainerPointIndicesWithId >* leaf_node =   static_cast< OctreeLeafNode<OctreeContainerPointIndicesWithId>* > (node);
  logLeafNodeContainerSize(leaf_node);

	int parentId = leaf_node->getContainer().getNodeId();
	int childrenCounter = leaf_node->getContainer().getSize();
	std::vector<double> featuresCoefficients;

	// Iterate through container elements, i.e. cloud points.
	std::vector<int> point_indices;
	leaf_node->getContainer().getPointIndices(point_indices);
  
	for(unsigned int i=0; i<leaf_node->getContainer().getSize(); i++)
	{
		PointXYZSHOT p = cloud->at(point_indices[i]);
    logPoint(p, point_indices[i]);
    string featureName = createFeatureName(p.pointId);
    string parentName = createVoxelName(parentId);
		addNode(featureName);
		addArc(featureName, parentName);
//		double coefficient = (double) p.multiplicity/(jointMultiplicityVector[i] * childrenCounter * 2);
		double coefficient = (double) p.multiplicity/(childrenCounter * 2);
		featuresCoefficients.push_back(coefficient);
	}//: for points		
  
	LOG(LDEBUG) << "voxel ID: " << parentId;
	LOG(LDEBUG) << "voxel name: " << createVoxelName(parentId);
	LOG(LDEBUG) << "children count: " <<childrenCounter;
  setVoxelNodeCPT(parentId, featuresCoefficients, childrenCounter);
	leafNodeCount++;
}

void CreateNetworkSHOT::createChild(pcl::octree::OctreeNode* child, int parentId)
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

void CreateNetworkSHOT::addVoxelNode(int id)
{
	stringstream name;
	name << "V_" << id;
	string voxelName = name.str();
  addNode(voxelName);
}

string CreateNetworkSHOT::createVoxelName(int id)
{
	stringstream name;
	name << "V_" << id;
	string voxelName = name.str();
  return voxelName;
}

string CreateNetworkSHOT::createFeatureName(int id)
{
	stringstream name;
	name << "F_" << id;
	string featureName(name.str());
	return featureName;
}

//TODO: make functionality more general
void CreateNetworkSHOT::setVoxelNodeCPT(int id, std::vector<double> featuresCoefficients, int childrenCounter) 
{
  string voxelName = createVoxelName(id);
	setNodeCPT(voxelName, featuresCoefficients);
	//            setNodeCPT(voxelName, childrenCounter);
}

void CreateNetworkSHOT::addNode(std::string name)
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

void CreateNetworkSHOT::addArc(string parentName, string childName)
{
	int childNode = theNet.FindNode(childName.c_str());
	int parentNode = theNet.FindNode(parentName.c_str());
	theNet.AddArc(parentNode, childNode);
}

//TODO: split into two methods: calculating probability and actually setting node's cpt
void CreateNetworkSHOT::setNodeCPT(string name, int numberOfParents)
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

void CreateNetworkSHOT::setNodeCPT(string name, std::vector<double> parentsCoefficients)
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

void CreateNetworkSHOT::fillCPT(string name, std::vector<double> probabilities)
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

int CreateNetworkSHOT::generateNext(std::string::iterator start, std::string::iterator end)
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

string CreateNetworkSHOT::getNodeName(int nodeHandle)
{
    return features[nodeHandle];
}

void CreateNetworkSHOT::loadNetwork()
{
    int result = -1;
    //result = theNet.ReadFile("/home/qiubix/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    //result = theNet.ReadFile("/home/kkaterza/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    LOG(LWARNING) << "Reading network file: " << result;
}

void CreateNetworkSHOT::logLeafNodeContainerSize(OctreeLeafNode<OctreeContainerPointIndicesWithId> *leaf_node)
{
	int containter_size = leaf_node->getContainer().getSize();
	if(containter_size >8) {
		LOG(LWARNING) << "Leaf containter has big number of features! (" << containter_size << ")";
	}//: if
	if(containter_size > maxLeafContainerSize)
    maxLeafContainerSize = containter_size;
}

int CreateNetworkSHOT::sumMultiplicityInsideVoxel(pcl::octree::OctreeLeafNode<OctreeContainerPointIndicesWithId> *leaf_node)
{
  int summedFeaturesMultiplicity = 0;
	std::vector<int> point_indices;
	leaf_node->getContainer().getPointIndices(point_indices);
	for(unsigned int j=0; j<leaf_node->getContainer().getSize(); j++) {
		PointXYZSHOT p = cloud->at(point_indices[j]);
		summedFeaturesMultiplicity += p.multiplicity;
	}
  return summedFeaturesMultiplicity;
}

void CreateNetworkSHOT::logPoint(PointXYZSHOT p, int index)
{
		LOG(LTRACE) << "Point index = " << index;
		LOG(LTRACE) << "p.x = " << p.x << " p.y = " << p.y << " p.z = " << p.z;
		LOG(LTRACE) << "multiplicity: " << p.multiplicity;
		LOG(LTRACE) << "pointId " << p.pointId;
}

void CreateNetworkSHOT::mapFeaturesNames()
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
