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
  coordinates.GoFirst();
  int position = coordinates.GoToCurrentPosition();
  while(position != DSL_OUT_OF_RANGE) {
    probabilities.push_back(coordinates.UncheckedValue());
    position = coordinates.Next();
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
  coordinates.GoFirst();
  int position = coordinates.GoToCurrentPosition();
  std::vector<double>::iterator it = probabilities.begin();

  for ( ; position != DSL_OUT_OF_RANGE && it != probabilities.end(); ++it, position = coordinates.Next() ) {
    if (*it < 0 || *it > 1)
      throw IncorrectProbabilityValueException();
    coordinates.UncheckedValue() = *it;
    LOG(LTRACE) << "Probability: " << *it;
  }
}

double CPTManager::getProbability()
{
  return 0.6;
}

} //: namespace Network
} //: namespace Processors
