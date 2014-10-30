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

    h_onNetwork.setup(this, &SOMEvaluation::onNetwork);
    registerHandler("onNetwork", &h_onNetwork);
    registerStream("in_networks", &in_networks);
    addDependency("onNetwork", &in_networks);

    h_onInstance.setup(this, &SOMEvaluation::onInstance);
    registerHandler("onInstance", &h_onInstance);
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
}

void SOMEvaluation::onInstance()
{
	LOG(LDEBUG) << "SOMEvaluation::onInstance";
	if(theNet.GetNumberOfNodes() != 0) {
		instance = in_instanceMatchedFeatures.read();
		evaluate();
	}
}

void SOMEvaluation::evaluate()
{
		LOG(LDEBUG) << "================= SOMEvaluation: evaluate =================";
		LOG(LDEBUG) << "instance size: " << instance.size();

		theNet = networks[0];

		Common::Timer timer;
		timer.restart();

		theNet.ClearAllEvidence();
    deactivateFeatures();
		activateMatchedFeatureNodes();
		theNet.UpdateBeliefs();

		displayHypothesisProbability();

		LOG(LINFO) << " runtime: " << timer.elapsed();
    LOG(LDEBUG) << "SOMEvaluation finished";
}

void SOMEvaluation::deactivateFeatures()
{
	int nodeId = 0;
	while(true) {
		std::stringstream ss;
		ss << "F_" << nodeId;
		std::string nodeName(ss.str());
		int node = theNet.FindNode(nodeName.c_str());
		if(node != DSL_OUT_OF_RANGE) {
			theNet.GetNode(node)->Value()->SetEvidence(1);
		}
		else {
			break;
		}
		nodeId++;
	}
}

void SOMEvaluation::activateMatchedFeatureNodes()
{
		for (unsigned i=0; i<instance.size(); ++i) {
				int node = findFeatureNode(instance[i]);
				LOG(LDEBUG) << "Observing node: nodeId = " << node;
				if(node != DSL_OUT_OF_RANGE) {
						theNet.GetNode(node)->Value()->SetEvidence(0);
				}
		}
    LOG(LDEBUG) << "Finished activating matched features";
}

void SOMEvaluation::displayHypothesisProbability(int modelId)
{
  string nodeName = "H_" + modelId;
  int hypothesis = theNet.FindNode(nodeName);
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
		theNet.GetNode(nodeId)->Value();
		DSL_sysCoordinates theCoordinates(*theNet.GetNode(nodeId)->Value());
		DSL_idArray *theNames = theNet.GetNode(nodeId)->Definition()->GetOutcomesNames();
		theCoordinates[0] = theNames->FindPosition("YES");
		theCoordinates.GoToCurrentPosition();
		double probability = theCoordinates.UncheckedValue();
		return probability;
}

}//: namespace Network
}//: namespace Processors
