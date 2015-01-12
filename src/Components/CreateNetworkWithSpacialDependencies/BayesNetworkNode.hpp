#ifndef BAYES_NETWORK_NODE_HPP
#define BAYES_NETWORK_NODE_HPP

#include <string>
#include "../../../lib/SMILE/smile.h"

namespace Processors {
namespace Network {

class BayesNetworkNode {
public:
  BayesNetworkNode(DSL_node* node);
  ~BayesNetworkNode() {}

  bool isVisited();
  void visitNode();

  std::string getName();
private:
  DSL_node* node;
  int nodeHandle;
  bool visited;
  std::string nodeName;
};

}//: namespace Processors
}//: namespace Network

#endif //BAYES_NETWORK_NODE_HPP
