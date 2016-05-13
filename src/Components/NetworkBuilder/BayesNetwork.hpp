#pragma once
#ifndef BAYES_NETWORK_HPP
#define BAYES_NETWORK_HPP

#include <string>
#include <vector>
#include <gtest/gtest.h>

#include "../../../lib/SMILE/smile.h"

#include "BayesNetworkNode.hpp"
#include "AbstractNetwork.hpp"

namespace Processors {
namespace Network {

class BayesNetwork : public AbstractNetwork {
public:
  BayesNetwork();
  ~BayesNetwork() {}

  double getNodeProbability(const std::string& name);
  void clearEvidence();
  bool nodeExists(const std::string& nodeName);
  void setNodeEvidence(const std::string& nodeName, int state);
  void propagateProbabilities();

  bool isEmpty();
  bool hasNode(const char* nodeName);
  //building network
  //TODO: extract BayesNetworkBuilder class
  void addVoxelNode(int id);
  void addFeatureNode(int id);
  std::string createVoxelName(int id);
  std::string createFeatureName(int id);
  void connectNodes(std::string parentName, std::string childName);

  //manipulating CPTs
  void setCPTofAllVoxelNodes(unsigned int numberOfVoxels);
  void setNodeCPT(std::string name, int numberOfParents);

  //getters
  std::string getNodeName(int nodeHandle);
  int getNumberOfChildren(const char* nodeName);
  int getNumberOfNodes();
  int getNumberOfFeatureNodes();
  std::vector<std::string> getFeatureNodeNames();
  BayesNetworkNode getNode(std::string name);
  BayesNetworkNode getNextRootNode();
  BayesNetworkNode getChild(BayesNetworkNode parent);
  bool visitNode(BayesNetworkNode& node);
  int getNodeEvidence(const std::string& featureNodeName);

  //exporting network
  void exportNetworkToFile();
  DSL_network getNetwork();
  void setNetwork(DSL_network network);

private:
  DSL_network network;
  std::vector <BayesNetworkNode> featureNodes;
  int nextRootNodePosition;

  //TODO: wrap network node in separate class
  void addNode(std::string name);

  int generateNext(std::string::iterator start, std::string::iterator end);

  FRIEND_TEST(BayesNetworkTest, shouldFillNodeCPT);
};


}//: namespace Network
}//: namespace Processors

#endif //BAYES_NETWORK_HPP
