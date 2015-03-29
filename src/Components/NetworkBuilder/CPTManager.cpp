#include "CPTManager.hpp"
#include "Logger.hpp"
#include "CPTManagerExceptions.hpp"

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

void CPTManager::fillCPT(std::vector<double> probabilities)
{
  std::string name(node->Info().Header().GetId());
  LOG(LTRACE) << "Filling CPT of node " << name;

  int cptSize = node->Definition()->GetSize();
  if (cptSize != probabilities.size() ) {
    throw DivergentCPTSizeException();
  }

  DSL_sysCoordinates coordinates(*node->Definition());
  std::vector<double>::iterator it = probabilities.begin();
  //TODO: switch do-while to for.
  // this is somehow problematic because Next() method automatically increments coordinates
  do {
    coordinates.UncheckedValue() = *it;
    LOG(LTRACE) << "Probability: " << *it;
    ++it;
  } while(coordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

} //: namespace Network
} //: namespace Processors
