#include <sstream>
#include <iostream>

#include "BayesNetwork.hpp"
#include "Logger.hpp"

namespace Processors {
namespace Network {

void BayesNetwork::addVoxelNode(int id)
{
  LOG(LTRACE) << "Adding voxel node to network";
  std::string voxelName = createVoxelName(id);
  addNode(voxelName);
}

void BayesNetwork::addFeatureNode(int id)
{
  LOG(LTRACE) << "Adding feature node to network";
  std::string featureName = createFeatureName(id);
  addNode(featureName);
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

void BayesNetwork::addArc(std::string parentName, std::string childName)
{
  LOG(LTRACE) << "Adding arc between nodes: " << parentName << "->" << childName;
  int childNode = network.FindNode(childName.c_str());
  int parentNode = network.FindNode(parentName.c_str());
  network.AddArc(parentNode, childNode);
}

//FIXME: change confusing names - children or ancestors? 
void BayesNetwork::setCPTofAllVoxelNodes(unsigned int numberOfVoxels)
{
  LOG(LTRACE) << "Setting CPTs of all nodes in network";
  for (unsigned int i=0; i<numberOfVoxels; ++i) {
    LOG(LTRACE) << "Setting CPT of voxel number " << i << " " << createVoxelName(i);
    int numberOfChildren = network.NumParents(i);
    LOG(LTRACE) << "Number of ancestors: " << numberOfChildren;
    if(numberOfChildren == 0) 
      continue;
    std::string nodeName = createVoxelName(i);
    setNodeCPT(nodeName, numberOfChildren);
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

  fillCPT(name, probabilities);
}

std::string BayesNetwork::getNodeName(int nodeHandle)
{
  LOG(LTRACE) << "Get name of node: " << nodeHandle;
}

int BayesNetwork::getNumberOfChildren(int nodeId)
{
  LOG(LTRACE) << "Get number of children of node nr: " << nodeId;
}

int BayesNetwork::getNumberOfNodes()
{
  return network.GetNumberOfNodes();
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
  LOG(LDEBUG) << "Add node to network: " << name;
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
