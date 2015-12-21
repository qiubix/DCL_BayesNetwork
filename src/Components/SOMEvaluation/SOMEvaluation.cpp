/*!
 * \file SOMEvaluation.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>
#include <Common/Timer.hpp>

#include "SOMEvaluation.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

SOMEvaluation::SOMEvaluation(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello SOMEvaluation";
}

SOMEvaluation::~SOMEvaluation()
{
  LOG(LTRACE)<<"Good bye SOMEvaluation";
}

void SOMEvaluation::prepareInterface()
{
  LOG(LTRACE) << "SOMEvaluation::prepareInterface";

  registerHandler("onNetwork", boost::bind(&SOMEvaluation::onNetwork, this));
  registerStream("in_networks", &in_networks);
  addDependency("onNetwork", &in_networks);

  registerHandler("onInstance", boost::bind(&SOMEvaluation::onInstance, this));
  registerStream("in_instanceMatchedFeatures", &in_instanceMatchedFeatures);
  addDependency("onInstance", &in_instanceMatchedFeatures);

  registerStream("out_probabilities", &out_probabilities);
}

bool SOMEvaluation::onInit()
{
  LOG(LTRACE) << "SOMEvaluation::initialize";
  return true;
}

bool SOMEvaluation::onFinish()
{
  LOG(LTRACE) << "SOMEvaluation::finish";
  return true;
}

bool SOMEvaluation::onStart()
{
  LOG(LTRACE) << "SOMEvaluation::onStart";
  return true;
}

bool SOMEvaluation::onStop()
{
  LOG(LTRACE) << "SOMEvaluation::onStop";
  return true;
}

void SOMEvaluation::onNetwork()
{
  LOG(LWARNING) << "SOMEvaluation::onNetwork";
  networks = in_networks.read();
  //theNet = networks[0];
  setNetwork(networks[0]);
}

void SOMEvaluation::onInstance()
{
  LOG(LDEBUG) << "SOMEvaluation::onInstance";
  if( ! network -> isEmpty()) {
    LOG(LDEBUG) << "There is a network ready to be evaluated";
    instance = in_instanceMatchedFeatures.read();
    evaluate();
  }
}

void SOMEvaluation::evaluate()
{
  LOG(LDEBUG) << "================= SOMEvaluation: evaluate =================";
  LOG(LDEBUG) << "instance size: " << instance.size();

  //theNet = networks[0];

  Common::Timer timer;
  timer.restart();

  LOG(LDEBUG) << "clearing evidence...";
  network -> clearEvidence();
//  theNet.UpdateBeliefs();
//  theNet.ClearAllEvidence();
//  theNet.UpdateBeliefs();
  deactivateFeatures();
  activateMatchedFeatureNodes();
  network -> propagateProbabilities();
  //theNet.UpdateBeliefs();

//  displayHypothesisProbability();

  LOG(LINFO) << " runtime: " << timer.elapsed();
  LOG(LDEBUG) << "SOMEvaluation finished";
}

void SOMEvaluation::deactivateFeatures()
{
  LOG(LTRACE) << "Deactivating all features";
  int nodeId = 0;
  while(true) {
    std::stringstream ss;
    ss << "F_" << nodeId;
    std::string nodeName(ss.str());
    if ( network -> nodeExists(nodeName) ) {
      network -> setNodeEvidence(nodeName, 0);
    }
//    int node = theNet.FindNode(nodeName.c_str());
//    if(node != DSL_OUT_OF_RANGE) {
//      LOG(LWARNING) << "Deactivating node " << nodeName;
//      theNet.GetNode(node)->Value()->SetEvidence(0);
//      theNet.UpdateBeliefs();
//    }
    else {
      break;
    }
    nodeId++;
  }
}

void SOMEvaluation::activateMatchedFeatureNodes()
{
  LOG(LTRACE) << "Activating matched feature nodes";
  for (unsigned i =0; i <instance.size(); ++i) {
    std::stringstream ss;
    ss << "F_" << instance[i];
    std::string nodeName(ss.str());
    network -> setNodeEvidence(nodeName, 1);
    /*
    int node = findFeatureNode(instance[i]);
    LOG(LDEBUG) << "Observing node: nodeId = " << node << " point id: " << instance[i];
    if(node != DSL_OUT_OF_RANGE) {
      LOG(LWARNING) << "Activating node " << instance[i];
      LOG(LWARNING) << "CPT size " << theNet.GetNode(node)->Definition()->GetSize();
      LOG(LWARNING) << "children " << theNet.GetChildren(node).GetSize();
      LOG(LWARNING) << "parents " << theNet.GetParents(node).GetSize();
      LOG(LWARNING) << "children " << theNet.NumChildren(node);
      LOG(LWARNING) << "parents " << theNet.NumParents(node);
      //LOG(LWARNING) << "parents " << theNet.GetNode(node)->Parents().GetSize();
      theNet.GetNode(node)->Value()->SetEvidence(1);
      theNet.UpdateBeliefs();
    }
     */
  }
  LOG(LDEBUG) << "Finished activating matched features";
}

void SOMEvaluation::displayHypothesisProbability(int modelId)
{
  //TODO: adapt for multiple models
  //string nodeName = "H_" + modelId;
  string nodeName = "V_0";
  LOG(LTRACE) << "Display probability of hypothesis: " << nodeName;
  int hypothesis = theNet.FindNode(nodeName.c_str());
  double hypothesisProbability = getNodeProbability(hypothesis);

  LOG(LWARNING) << "Hypothesis probability: " << hypothesisProbability;

  hypothesesProbabilities.push_back(hypothesisProbability);
  out_probabilities.write(hypothesesProbabilities);
}

int SOMEvaluation::findFeatureNode(int nodeId)
{
  std::stringstream ss;
  ss << "F_" << nodeId;
  std::string nodeName(ss.str());
  return theNet.FindNode(nodeName.c_str());
}

double SOMEvaluation::getNodeProbability(int nodeId)
{
//  LOG(LTRACE) << "Get probability of node with id = " << nodeId;
//  DSL_sysCoordinates theCoordinates(*theNet.GetNode(nodeId)->Value());
//  DSL_idArray *theNames = theNet.GetNode(nodeId)->Definition()->GetOutcomesNames();
//  theCoordinates[0] = theNames->FindPosition("YES");
//  theCoordinates.GoToCurrentPosition();
//  double probability = theCoordinates.UncheckedValue();
  double probability = network -> getNodeProbability("V_0");
  return probability;
}

void SOMEvaluation::setNetwork(AbstractNetwork* network)
{
  this -> network = network;
}

void SOMEvaluation::setInstance(std::vector<int> instance) {
  this -> instance = instance;
}

}//: namespace Network
}//: namespace Processors
