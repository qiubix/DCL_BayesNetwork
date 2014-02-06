/*!
 * \file CreateNetwork.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "CreateNetwork.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace pcl::octree;

namespace Processors {
namespace Network {

class OctreeContainerEmptyWithId : public OctreeContainerEmpty 
{
public:
    OctreeContainerEmptyWithId() {
        this->nodeId = -1;
        this->parentId = -1;
    }

    int getNodeId()
    {
        return nodeId;
    }
    int getParentId()
    {
       	return parentId;
    }
    void setNodeId(int nodeId)
    {
        this->nodeId = nodeId;
    }
    void setParentId(int parentId) 
    {
        this->parentId = parentId;
    }

private:
    int nodeId;
    int parentId;
};


CreateNetwork::CreateNetwork(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello CreateNetwork\n";
}

CreateNetwork::~CreateNetwork()
{
    LOG(LTRACE)<<"Good bye CreateNetwork\n";
}

void CreateNetwork::prepareInterface()
{
    LOG(LTRACE) << "CreateNetwork::prepareInterface\n";

//    h_onModels.setup(this, &CreateNetwork::onModels);
//    registerHandler("onModels", &h_onModels);
//    h_onJointMultiplicity.setup(this, &CreateNetwork::onJointMultiplicity);
//    registerHandler("onJointMultiplicity", &h_onJointMultiplicity);

//    registerStream("in_models", &in_modelsMultiplicity);
//    addDependency("onModels", &in_modelsMultiplicity);
//    registerStream("in_jointMultiplicity", &in_jointMultiplicity);
//    addDependency("onJointMultiplicity", &in_jointMultiplicity);

//    registerStream("out_network", &out_network);
	// Register data streams.
//	registerStream("in_cloud", &in_cloud_xyz);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	// Register handlers
	h_cloud_xyzrgb_to_octree.setup(boost::bind(&CreateNetwork::cloud_xyzsift_to_octree, this));
	registerHandler("cloud_xyzrgb_to_octree", &h_cloud_xyzrgb_to_octree);
	addDependency("cloud_xyzrgb_to_octree", &in_cloud_xyzsift);
}

DSL_network CreateNetwork::getNetwork()
{
    return this->theNet;
}

string CreateNetwork::getNodeName(int nodeHandle)
{
    return features[nodeHandle];
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

void CreateNetwork::onModels()
{
    LOG(LDEBUG) << "CreateNetwork::onModels";
//    map<int,int> newModel = in_models.read();
//    models.push_back(newModel);
    models = in_modelsMultiplicity.read();
    LOG(LDEBUG) << "CreateNetwork: Number of models: " << models.size();
}

void CreateNetwork::onJointMultiplicity()
{
    LOG(LDEBUG) << "CreateNetwork::onJointMultiplicity";
	jointMultiplicityVector = in_jointMultiplicity.read();
    //TODO: FIXME: think of better way of synchronizing input dataports
    if (theNet.GetNumberOfNodes() == 0 && !models.empty()) {
		mapFeaturesNames();
		buildNetwork();
		exportNetwork();
    }
}

//void CreateNetwork::buildNetwork()
//{
//    LOG(LDEBUG) << "=========== BUILDING NETWORK ==============";
//    vector<string> outcomesNames;
//    vector<string> parentsNames;
//    outcomesNames.push_back("YES");
//    outcomesNames.push_back("NO");
//    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
//        addNode(features[i], outcomesNames, parentsNames);
//        LOG(LTRACE) << "Added feature no " << i;
//    }
//    LOG(LDEBUG) << "Number of feature nodes added: " << jointMultiplicityVector.size();
//    for (unsigned j=0; j<models.size(); ++j) {
//        stringstream name;
//        name << "H" << j;
//        string hypothesisName(name.str());
//        map<int,int> modelFeatures = models[j];
//        map<int,int>::iterator it = modelFeatures.begin();
//        while (it != modelFeatures.end()) {
//            string parentName = features[it->first];
//            parentsNames.push_back(parentName);
//            ++it;
//        }
//        addNode(hypothesisName, outcomesNames, parentsNames);
//        LOG(LTRACE) << "Added hypotheses no " << j;
//        parentsNames.clear();
//    }
//    LOG(LDEBUG) << "Added " << models.size() << " hypotheses nodes";
//    setBaseNetworkCPTs();
//}

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

//void CreateNetwork::setBaseNetworkCPTs()
//{
//    LOG(LDEBUG) << "Set base network CPTs";
//    setBaseFeaturesCPTs();
////    setBaseHypothesesCPTs();
//}

//void CreateNetwork::setBaseFeaturesCPTs()
//{
//    LOG(LDEBUG) << "------- Set base features CPTs -------";
//    int sum = 0;
//    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
//        LOG(LDEBUG) << "feature " << i << " multiplicity: " << jointMultiplicityVector[i];
//        sum += jointMultiplicityVector[i];
//    }
//    LOG(LDEBUG) << "summed multiplicity of all features: " << sum;

//    std::map<int,string>::iterator it = features.begin();
//    for( ; it!=features.end(); ++it) {
//        LOG(LDEBUG) << "feature no: " << it->first;
//        int multiplicity = jointMultiplicityVector[it->first];
//        LOG(LDEBUG) << "feature multiplicity in model: " << multiplicity;
//        double baseProbability = (double) multiplicity/sum;
//		std::vector <double> probabilities;
//        probabilities.push_back(baseProbability);
//        probabilities.push_back(1 - baseProbability);
//        LOG(LDEBUG) << "calculated base probability " << baseProbability;
//        setNodeCPT(it->second, probabilities);
//    }
//}

//void CreateNetwork::setBaseHypothesesCPTs()
//{
//    LOG(LDEBUG) << "------ Set base hypotheses CPTs ------";
//    int jointMultiplicitySum = 0;
//    for (unsigned i=0; i<jointMultiplicityVector.size(); ++i) {
//        jointMultiplicitySum += jointMultiplicityVector[i];
//    }

//    for (unsigned i=0; i<models.size(); ++i) {
//        LOG(LDEBUG) << "hypothesis no :" << i;
//        double P_Hi = (double) 1/models.size();
//        map<int,int> modelFeatures = models[i];
//        map<int,int>::iterator k = modelFeatures.begin();

//        int modelMultiplicitySum = 0;
//        while (k != modelFeatures.end()) {
//            modelMultiplicitySum += k->second;
//            ++k;
//        }

//        k = modelFeatures.begin();
//        double baseProbability = 1;
//        while (k != modelFeatures.end()) {
//            double P_Fk_given_Hi = (double) (k->second)/modelMultiplicitySum;
//            double P_Fk = (double) jointMultiplicityVector[k->first]/jointMultiplicitySum;
//            double P_Hi_given_Fk = (double) P_Fk_given_Hi * P_Hi / P_Fk;
//            baseProbability *= P_Hi_given_Fk;
//            ++k;
//        }
//        LOG(LDEBUG) << "Calculated probability: " << baseProbability;
//		std::vector<double> probabilities;
//        probabilities.push_back(baseProbability);
//        probabilities.push_back(1 - baseProbability);
//        stringstream name;
//        name << "H" << i;
//        string hypothesisName(name.str());
//        setNodeCPT(hypothesisName, probabilities);
//    }
//}

void CreateNetwork::addNode(const std::string name, const std::vector<string> parentsNames)
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

    int nextParent;
    for (int i=0; i<parentsNames.size(); i++) {
        nextParent = theNet.FindNode(parentsNames[i].c_str());
        theNet.AddArc(nextParent, newNode);
    }
//    features.insert(std::make_pair<int,string>(newNode, name));
}

void CreateNetwork::setNodeCPT(const string name, vector<double> probabilities)
{
    LOG(LDEBUG) << "Set node CPT: " << name;
    int node = theNet.FindNode(name.c_str());
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(node)->Definition());

    std::vector<double>::iterator it = probabilities.begin();
    do {
        theCoordinates.UncheckedValue() = *it;
        ++it;
    } while(theCoordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

void CreateNetwork::exportNetwork()
{
    theNet.WriteFile("out_network.xdsl", DSL_XDSL_FORMAT);
    out_network.write(theNet);
}

//void CreateNetwork::buildNetworkForModel(pcl::octree::OctreePointCloud<PointXYZSIFT> octree, int modelId)
//{
//	DSL_network theNet;
//	OctreePointCloud<PointXYZSIFT>::BreadthFirstIterator bfIt;
//	const OctreePointCloud<PointXYZSIFT>::BreadthFirstIterator bfIt_end = octree.breadth_end();

//	stringstream name;
//	name << "H_" << modelId;
//	string hypothesisName(name.str());
//	std::vector<string> parentsNames;
//	addNode(theNet, hypothesisName, parentsNames);

//	for (bfIt = octree.breadth_begin(); bfIt != bfIt_end; ++bfIt) {
//		OctreeNode* node = bfIt.getCurrentOctreeNode(); 
        
//        unsigned char child_idx;
//		if (node->getNodeType () == BRANCH_NODE) 
//		{
//			OctreeBranchNode<PointXYZSIFT>* branch_node = static_cast<OctreeBranchNode<PointXYZSIFT>*> (node);
//            std::vector<string> childrenNames;
//			// iterate over all children
//			for (child_idx = 0; child_idx < 8 ; ++child_idx)
//			{
//				// if child exist
//                if (branch_node->hasChild(child_idx)) {
//                    OctreeNode* child_node = branch_node->getChildPtr(child_idx);
                    
//                }
//			}
//		}
//	}
//}


void CreateNetwork::cloud_xyzsift_to_octree() {
	LOG(LTRACE) << "PC2Octree::cloud_xyzsift_to_octree";
	// Read from dataport.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

	// Set voxel resolution.
	float voxelSize = 0.01f;
	OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndices, OctreeContainerEmptyWithId> octree (voxelSize);
	// Set input cloud.
	octree.setInputCloud(cloud);
	// Calculate bounding box of input cloud.
	octree.defineBoundingBox();

	// Add points from input cloud to octree.
	octree.addPointsFromInputCloud ();


	unsigned int lastDepth = 0;
	unsigned int branchNodeCount = 0;
	unsigned int leafNodeCount = 0;
	unsigned int maxLeafContainerSize = 0;

	// Use breadth-first iterator
	OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndices, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt;
	const OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndices, OctreeContainerEmptyWithId>::BreadthFirstIterator bfIt_end = octree.breadth_end();

	for (bfIt = octree.breadth_begin(); bfIt != bfIt_end; ++bfIt)
	{
		LOG(LINFO) << "depth = " << bfIt.getCurrentOctreeDepth ();
		pcl::octree::OctreeNode* node = bfIt.getCurrentOctreeNode(); 
		if (node->getNodeType () == BRANCH_NODE) {
			// current node is a branch node
			LOG(LINFO) << "BRANCH";

			OctreeBranchNode<OctreeContainerEmptyWithId>* branch_node = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>* > (node);
			// iterate over all children
			unsigned char child_idx;
			for (child_idx = 0; child_idx < 8 ; ++child_idx) {
				if (branch_node->hasChild(child_idx)) {
					LOG(LINFO) << "Child "<<(int)child_idx << "present";
//					OctreeBranchNode* current_branch = octree->getBranchChildPtr(*current_branch, child_idx);
                    OctreeNode* child = branch_node->getChildPtr(child_idx);
                    if(child->getNodeType() == BRANCH_NODE) {
						OctreeBranchNode<OctreeContainerEmptyWithId>* child_node = static_cast<OctreeBranchNode<OctreeContainerEmptyWithId>*> (child);
                        LOG(LWARNING) << "node id: " << child_node->getContainer().getNodeId();
                    }
				}
			}
			branchNodeCount++;
		}

		if (node->getNodeType () == LEAF_NODE) 
		{
			// Current node is a branch node.
			LOG(LINFO) << "LEAF";
			// Cast to proper data structure.
			OctreeLeafNode< OctreeContainerPointIndices >* leaf_node =   static_cast< OctreeLeafNode<OctreeContainerPointIndices>* > (node);
			// Get container size.
			int containter_size = leaf_node->getContainer().getSize();
			// Check whether size is proper.
			if(containter_size >8) {
				LOG(LERROR) << "Leaf containter too big! (" << containter_size << ")";
			}//: if
			// Get maximum size of container.	
			if(containter_size > maxLeafContainerSize)
				maxLeafContainerSize = containter_size;
	
			// Iterate through container elements, i.e. cloud points.
			std::vector<int> point_indices;
	 		leaf_node->getContainer().getPointIndices(point_indices);
			for(unsigned int i=0; i<leaf_node->getContainer().getSize(); i++)
			{
				LOG(LDEBUG) << "Iteration number " << i << " Point index=" << point_indices[i];
				PointXYZSIFT p = cloud->at(point_indices[i]);
				LOG(LINFO) << "p.x = " << p.x << " p.y = " << p.y << " p.z = " << p.z;
				LOG(LINFO) << "multiplicity: " << p.multiplicity;
			}//: for points		
			leafNodeCount++;
		}//: if leaf
	}//: for nodes

	LOG(LWARNING) << "ELO! branchNodeCount: " << branchNodeCount;
	LOG(LWARNING) << "ELO! leafNodeCount: " << leafNodeCount;
	LOG(LWARNING) << "ELO! maxLeafContainerSize: " << maxLeafContainerSize;
    
    theNet.WriteFile("out_network.xdsl", DSL_XDSL_FORMAT);

}

}//: namespace Network
}//: namespace Processors
