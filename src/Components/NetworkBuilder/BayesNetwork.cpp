#include <sstream>
#include <iostream>

#include "BayesNetwork.hpp"
#include "Logger.hpp"
#include "CPTManager.hpp"

namespace Processors {
namespace Network {

BayesNetwork::BayesNetwork()
{
  network.SetDefaultBNAlgorithm(DSL_ALG_BN_LAURITZEN);
  nextRootNodePosition = 0;
}

bool BayesNetwork::isEmpty()
{
  return network.GetNumberOfNodes() == 0;
}

bool BayesNetwork::hasNode(const char* nodeName)
{
  int nodeId = network.FindNode(nodeName);
  return nodeId != DSL_OUT_OF_RANGE;
}

void BayesNetwork::addVoxelNode(int id)
{
  std::string voxelName = createVoxelName(id);
  LOG(LDEBUG) << "Adding voxel node to network: " << voxelName;
  addNode(voxelName);
}

void BayesNetwork::addFeatureNode(int id)
{
  std::string featureName = createFeatureName(id);
  LOG(LDEBUG) << "Adding feature node to network: " << featureName;
  addNode(featureName);
  int featureNodeHandle = network.FindNode(featureName.c_str());
  BayesNetworkNode newNode(network.GetNode(featureNodeHandle));
  featureNodes.push_back(newNode);
}

std::string BayesNetwork::createVoxelName(int id)
{
  LOG(LTRACE) << "Creating voxel name";
  std::stringstream name;
  name << "V_" << id;
  std::string voxelName = name.str();
  return voxelName;
}

std::string BayesNetwork::createFeatureName(int id)
{
  LOG(LTRACE) << "Creating feature name";
  std::stringstream name;
  name << "F_" << id;
  std::string featureName(name.str());
  return featureName;
}

void BayesNetwork::connectNodes(std::string parentName, std::string childName)
{
  LOG(LDEBUG) << "Adding arc between nodes: " << parentName << "->" << childName;
  int childNode = network.FindNode(childName.c_str());
  int parentNode = network.FindNode(parentName.c_str());
  network.AddArc(parentNode, childNode);
}

void BayesNetwork::setCPTofAllVoxelNodes(unsigned int numberOfVoxels)
{
  LOG(LDEBUG) << "Setting CPTs of all nodes in network";
  for (unsigned int i=0; i<numberOfVoxels; ++i) {
    LOG(LTRACE) << "Setting CPT of voxel number " << i << " " << createVoxelName(i);
    std::string nodeName = createVoxelName(i);
    int nodeId = network.FindNode(nodeName.c_str());
    int numberOfParents = network.NumParents(nodeId);
    LOG(LTRACE) << "Number of ancestors: " << numberOfParents;
    if(numberOfParents == 0)
      continue;
    setNodeCPT(nodeName, numberOfParents);
  }
}

void BayesNetwork::setNodeCPT(std::string name, int numberOfParents)
{
  LOG(LTRACE) << "Setting CPT of node " << name;
  std::vector<double> probabilities;
  std::string s(numberOfParents,'1');
  do {
    int ones = std::count(s.begin(),s.end(),'1');
    double probability = (double) ones/numberOfParents;
    probabilities.push_back(probability);
    probabilities.push_back(1-probability);
  } while(generateNext(s.begin(), s.end()));

  int nodeId = network.FindNode(name.c_str());
  CPTManager cptManager(network.GetNode(nodeId));
  cptManager.fillCPT(probabilities);
}

std::string BayesNetwork::getNodeName(int nodeHandle)
{
  LOG(LTRACE) << "Get name of node: " << nodeHandle;
}

//FIXME: change interface of this method for much more intuitive
int BayesNetwork::getNumberOfChildren(int nodeId)
{
  LOG(LTRACE) << "Get number of children of node nr: " << nodeId;
  int numberOfChildren = 0;
  int nodeHandle = network.FindNode(createVoxelName(nodeId).c_str());
  numberOfChildren = network.NumChildren(nodeHandle);
  return numberOfChildren;
}

int BayesNetwork::getNumberOfNodes()
{
  return network.GetNumberOfNodes();
}

BayesNetworkNode BayesNetwork::getNextRootNode()
{
  BayesNetworkNode node = featureNodes[nextRootNodePosition];
  return node;
}

BayesNetworkNode BayesNetwork::getChild(BayesNetworkNode parent)
{
  LOG(LTRACE) << "Get child of a node: " << parent.getName();
  int childHandle = parent.getChildHandle();
  BayesNetworkNode child(network.GetNode(childHandle));
  LOG(LTRACE) << "Child name: " << child.getName();
  return child;
}

bool BayesNetwork::visitNode(BayesNetworkNode& node)
{
  LOG(LTRACE) << "Visiting node: " << node.getName();
  bool found = false;
  for (int i=0; i<featureNodes.size(); ++i) {
    if (featureNodes[i].getName() == node.getName()) {
      found = true;
      featureNodes[i].visitNode();
      node.visitNode();
      if (nextRootNodePosition < featureNodes.size()-1)
        ++nextRootNodePosition;
    }
  }
  return found;
}

void BayesNetwork::exportNetworkToFile()
{
  LOG(LTRACE) << "Exporting network to file out_network.xdsl";
  network.WriteFile("out_network.xdsl", DSL_XDSL_FORMAT);
}

DSL_network BayesNetwork::getNetwork()
{
  return network;
}

void BayesNetwork::addNode(std::string name)
{
  LOG(LTRACE) << "Add node to network: " << name;
  int newNode = network.AddNode(DSL_CPT, name.c_str());
  DSL_idArray outcomes;
  std::vector<std::string> outcomesNames;
  outcomesNames.push_back("YES");
  outcomesNames.push_back("NO");
  for (int i=0; i<outcomesNames.size(); i++) {
    outcomes.Add(outcomesNames[i].c_str());
  }
  network.GetNode(newNode)->Definition()->SetNumberOfOutcomes(outcomes);
}

void BayesNetwork::fillCPT(std::string name, std::vector<double> probabilities)
{
  LOG(LTRACE) << "Filling CPT of node " << name;
  int node = network.FindNode(name.c_str());
  DSL_sysCoordinates theCoordinates(*network.GetNode(node)->Definition());

  std::vector<double>::iterator it = probabilities.begin();
  do {
    theCoordinates.UncheckedValue() = *it;
    LOG(LTRACE) << "Probability: " << *it;
    ++it;
  } while(theCoordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

/*!
 * Generate next string of 1 and 0 to find next cell in CPT
 */
int BayesNetwork::generateNext(std::string::iterator start, std::string::iterator end)
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

}//: namespace Network
}//: namespace Processors
