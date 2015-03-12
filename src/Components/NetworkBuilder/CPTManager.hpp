#ifndef CPT_MANAGER_HPP
#define CPT_MANAGER_HPP

#include <string>
#include <vector>
#include "../../../lib/SMILE/smile.h"

namespace Processors {
namespace Network {

class CPTManager {
public:
  CPTManager(DSL_node* node);
  ~CPTManager() {}
  void fillCPT(std::string name, std::vector<double> probabilities);
private:
  DSL_node* node;
};

} //: namespace Network
} //: namespace Processors

#endif //CPT_MANAGER_HPP
