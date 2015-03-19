#include "CPTManager.hpp"
#include "Logger.hpp"

namespace Processors {
namespace Network {

CPTManager::CPTManager(DSL_node* node) {
  this->node = node;
}

std::vector<double> CPTManager::displayCPT()
{
  std::vector<double> probabilities;
  DSL_sysCoordinates coordinates(*node->Definition());
  while(true) {
    probabilities.push_back(coordinates.UncheckedValue());
    int position = coordinates.Next();
    if(position == DSL_OUT_OF_RANGE)
      break;
  }
  return probabilities;
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
