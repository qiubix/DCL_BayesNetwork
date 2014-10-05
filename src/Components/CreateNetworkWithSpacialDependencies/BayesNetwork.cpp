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
  int childNode = network.FindNode(childName.c_str());
  int parentNode = network.FindNode(parentName.c_str());
  network.AddArc(parentNode, childNode);
}

void Processors::Network::BayesNetwork::setCPTofAllNodes()
{
  LOG(LTRACE) << "Setting CPTs of all nodes in network";
}

void Processors::Network::BayesNetwork::setNodeCPT(std::string name, int numberOfParents)
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

std::string Processors::Network::BayesNetwork::getNodeName(int nodeHandle)
{
  LOG(LTRACE) << "Get name of node: " << nodeHandle;
}

int Processors::Network::BayesNetwork::getNumberOfChildren(int nodeId)
{
  LOG(LTRACE) << "Get number of children of node nr: " << nodeId;
}

void Processors::Network::BayesNetwork::addNode(std::string name)
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

void Processors::Network::BayesNetwork::fillCPT(std::string name, std::vector<double> probabilities)
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
int Processors::Network::BayesNetwork::generateNext(std::basic_string::iterator start, std::basic_string::iterator end)
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
