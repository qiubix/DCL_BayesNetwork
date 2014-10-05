#ifndef BAYES_NETWORK_HPP
#define BAYES_NETWORK_HPP

#include "../../../lib/SMILE/smile.h"

namespace Processors {
namespace Network {
  
class BayesNetwork {
public:
    BayesNetwork() {}
    ~BayesNetwork() {}
    void addVoxelNode(int id);
private:
    DSL_network network;

};


}//: namespace Network   
}//: namespace Processors

#endif //BAYES_NETWORK_HPP
