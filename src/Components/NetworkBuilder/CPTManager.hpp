#pragma once
#ifndef CPT_MANAGER_HPP
#define CPT_MANAGER_HPP

#include <string>
#include <vector>
//#include "SMILE/smile.h"

class DSL_node;

namespace Processors {
namespace Network {

class CPTManager {
public:
  CPTManager(DSL_node* node);
  ~CPTManager() {}
  std::vector<double> displayCPT();
  void fillCPT(std::vector<double> probabilities);
  double getProbability();
private:
  DSL_node* node;
};

} //: namespace Network
} //: namespace Processors

#endif //CPT_MANAGER_HPP
