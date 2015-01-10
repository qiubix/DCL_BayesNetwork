/*!
 * \file MockEvaluator.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <time.h>
#include <Common/Timer.hpp>

#include "MockEvaluator.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

MockEvaluator::MockEvaluator(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello MockEvaluator";
}

MockEvaluator::~MockEvaluator()
{
    LOG(LTRACE)<<"Good bye MockEvaluator";
}

void MockEvaluator::prepareInterface()
{
    LOG(LTRACE) << "MockEvaluator::prepareInterface";

    h_onNetwork.setup(this, &MockEvaluator::onNetwork);
    registerHandler("onNetwork", &h_onNetwork);
    registerStream("in_networks", &in_networks);
    addDependency("onNetwork", &in_networks);

    h_onInstance.setup(this, &MockEvaluator::onInstance);
    registerHandler("onInstance", &h_onInstance);
    registerStream("in_instanceMatchedFeatures", &in_instanceMatchedFeatures);
    addDependency("onInstance", &in_instanceMatchedFeatures);

    registerStream("out_probabilities", &out_probabilities);
}

bool MockEvaluator::onInit()
{
    LOG(LTRACE) << "MockEvaluator::initialize";
    return true;
}

bool MockEvaluator::onFinish()
{
    LOG(LTRACE) << "MockEvaluator::finish";
    return true;
}

bool MockEvaluator::onStart()
{
    LOG(LTRACE) << "MockEvaluator::onStart";
    return true;
}

bool MockEvaluator::onStop()
{
    LOG(LTRACE) << "MockEvaluator::onStop";
    return true;
}

void MockEvaluator::onNetwork()
{
  LOG(LWARNING) << "MockEvaluator::onNetwork";
  networks = in_networks.read();
  theNet = networks[0];
}

void MockEvaluator::onInstance()
{
  LOG(LWARNING) << "MockEvaluator::onInstance";
  if(theNet.GetNumberOfNodes() != 0) {
    LOG(LWARNING) << "There is a network ready to be evaluated";
    instance = in_instanceMatchedFeatures.read();
    evaluate();
  }
}

void MockEvaluator::evaluate()
{
		LOG(LDEBUG) << "================= MockEvaluator: evaluate =================";
		LOG(LDEBUG) << "instance size: " << instance.size();

		//theNet = networks[0];

		Common::Timer timer;
		timer.restart();

		displayHypothesisProbability();

		LOG(LINFO) << " runtime: " << timer.elapsed();
    LOG(LDEBUG) << "MockEvaluator finished";
}

void MockEvaluator::activateMatchedFeatureNodes()
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

void MockEvaluator::displayHypothesisProbability(int modelId)
{
  double hypothesisProbability = 1.0;

  LOG(LWARNING) << "Hypothesis probability: " << hypothesisProbability;

  hypothesesProbabilities.push_back(hypothesisProbability);
  out_probabilities.write(hypothesesProbabilities);
}

}//: namespace Network
}//: namespace Processors
