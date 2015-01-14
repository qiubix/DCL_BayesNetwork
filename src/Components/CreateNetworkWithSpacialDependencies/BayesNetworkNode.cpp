#include "BayesNetworkNode.hpp"

namespace Processors {
namespace Network {

BayesNetworkNode::BayesNetworkNode(DSL_node* node) {
  this->node = node;
  visited = false;
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

}//: namespace Processors
}//: namespace Network
