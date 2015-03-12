#include "CPTManager.hpp"
#include "Logger.hpp"

namespace Processors {
namespace Network {

CPTManager::CPTManager(DSL_node* node) {
  this->node = node;
}

void CPTManager::fillCPT(std::string name, std::vector<double> probabilities)
{
  LOG(LTRACE) << "Filling CPT of node " << name;
  DSL_sysCoordinates theCoordinates(*node->Definition());

  std::vector<double>::iterator it = probabilities.begin();
  do {
    theCoordinates.UncheckedValue() = *it;
    LOG(LTRACE) << "Probability: " << *it;
    ++it;
  } while(theCoordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

} //: namespace Network
} //: namespace Processors
