#include "BayesNetworkNode.hpp"

namespace Processors {
namespace Network {

BayesNetworkNode::BayesNetworkNode(DSL_node* node) {
  this->node = node;
  visited = false;
  nodeName = node->GetId();
  nodeHandle = node->Handle();
}

bool BayesNetworkNode::isVisited() {
  return visited;
}

void BayesNetworkNode::visitNode() {
  visited = true;
}

std::string BayesNetworkNode::getName() {
  std::string name(node->GetId());
  //return "F_0";
  return name;
}

int BayesNetworkNode::getChildHandle()
{
  DSL_network* network = node->Network();
  DSL_intArray children = network->GetChildren(nodeHandle);
  //TODO: check if array has any elements at all
  int childHandle = children[0];
  return childHandle;
}

}//: namespace Processors
}//: namespace Network
