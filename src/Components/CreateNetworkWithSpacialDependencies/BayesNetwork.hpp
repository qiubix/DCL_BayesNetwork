#ifndef BAYES_NETWORK_HPP
#define BAYES_NETWORK_HPP

#include "../../../lib/SMILE/smile.h"

namespace Processors {
namespace Network {
  
class BayesNetwork {
public:
    BayesNetwork() {}
    ~BayesNetwork() {}
    
    //building network
    void addVoxelNode(int id);
    string createVoxelName(int id);
    string createFeatureName(int id);
    void addArc(std::string parentName, std::string childName);
    
    //manipulating CPTs
    void setCPTofAllNodes();
    void setNodeCPT(std::string name, int numberOfParents);
    void fillCPT(std::string name, std::vector<double> probabilities);
    int generateNext(std::string::iterator start, std::string::iterator end);
    
    //getters
    std::string getNodeName(int nodeHandle);
    int getNumberOfChildren(int nodeId);
private:
    DSL_network network;

};


}//: namespace Network   
}//: namespace Processors

#endif //BAYES_NETWORK_HPP