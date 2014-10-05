#ifndef BAYES_NETWORK_HPP
#define BAYES_NETWORK_HPP

#include <string>

#include "../../../lib/SMILE/smile.h"

namespace Processors {
namespace Network {
  
class BayesNetwork {
public:
  BayesNetwork() {}
  ~BayesNetwork() {}

  //building network
  void addVoxelNode(int id);
  std::string createVoxelName(int id);
  std::string createFeatureName(int id);
  void addArc(std::string parentName, std::string childName);

  //manipulating CPTs
  void setCPTofAllVoxelNodes(unsigned int numberOfVoxels);
  void setNodeCPT(std::string name, int numberOfParents);

  //getters
  std::string getNodeName(int nodeHandle);
  int getNumberOfChildren(int nodeId);
private:
  DSL_network network;

  void addNode(std::string name);

  void fillCPT(std::string name, std::vector<double> probabilities);
  int generateNext(std::string::iterator start, std::string::iterator end);
};


}//: namespace Network   
}//: namespace Processors

#endif //BAYES_NETWORK_HPP
