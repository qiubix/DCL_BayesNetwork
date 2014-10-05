#include "BayesNetwork.hpp"
#include "Logger.hpp"

void Processors::Network::BayesNetwork::addVoxelNode(int id)
{
  LOG(LTRACE) << "Adding voxel node to network";
}

std::string Processors::Network::BayesNetwork::createVoxelName(int id)
{
  LOG(LTRACE) << "Creating voxel name";
}

std::string Processors::Network::BayesNetwork::createFeatureName(int id)
{
  LOG(LTRACE) << "Creating feature name";
}

void Processors::Network::BayesNetwork::addArc(std::string parentName, std::string childName)
{
  LOG(LTRACE) << "Adding arc between nodes: " << parentName << "->" << childName;
}

void Processors::Network::BayesNetwork::setCPTofAllNodes()
{
  LOG(LTRACE) << "Setting CPTs of all nodes in network";
}

void Processors::Network::BayesNetwork::setNodeCPT(std::string name, int numberOfParents)
{
  LOG(LTRACE) << "Setting CPT of node " << name;
}

void Processors::Network::BayesNetwork::fillCPT(std::string name, std::vector<double> probabilities)
{
  LOG(LTRACE) << "Filling CPT of node " << name;
}

int Processors::Network::BayesNetwork::generateNext(std::basic_string::iterator start, std::basic_string::iterator end)
{
  // Generate next string of 1 and 0 to find next cell in CPT
}

std::string Processors::Network::BayesNetwork::getNodeName(int nodeHandle)
{
  LOG(LTRACE) << "Get name of node: " << nodeHandle;
}

int Processors::Network::BayesNetwork::getNumberOfChildren(int nodeId)
{
  LOG(LTRACE) << "Get number of children of node nr: " << nodeId;
}
