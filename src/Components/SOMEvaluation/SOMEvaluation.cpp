/*!
 * \file SOMEvaluation.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

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
//    h_onModels.setup(this, &SOMEvaluation::onModels);
//    registerHandler("onModels", &h_onModels);
    
//    registerStream("in_models", &in_models);
//    registerStream("in_jointMultiplicity", &in_jointMultiplicity);
//    addDependency("onModels", &in_models);
//    addDependency("onModels", &in_jointMultiplicity);
    
    h_onNetwork.setup(this, &SOMEvaluation::onNetwork);
    registerHandler("onNetwork", &h_onNetwork);
    registerStream("in_network", &in_network);
    addDependency("onNetwork", &in_network);
    
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

void SOMEvaluation::onModels()
{
    LOG(LTRACE) << "SOMEvaluation::onModels";
//    models = in_models.read();
//    jointMultiplicityVector = in_jointMultiplicity.read();
}

void SOMEvaluation::onNetwork()
{
    LOG(LWARNING) << "SOMEvaluation::onNetwork";
    theNet = in_network.read();
}

void SOMEvaluation::onInstance()
{
	LOG(LWARNING) << "SOMEvaluation::onInstance";
	if(theNet.GetNumberOfNodes() != 0) {
		instance = in_instanceMatchedFeatures.read();
		evaluate();
	}
}

void SOMEvaluation::evaluate()
{
	LOG(LTRACE) << "SOMEvaluation::evaluate";
    LOG(LWARNING) << "instance size: " << instance.size();
    for (unsigned i=0; i<instance.size(); ++i) {
        int nodeId = instance[i];
        std::stringstream ss;
        ss << "F_" << nodeId;
        std::string nodeName(ss.str());
        int node = theNet.FindNode(nodeName.c_str());
        LOG(LWARNING) << "Observing node" << nodeName << ": nodeId = " << node;
        if(node != DSL_OUT_OF_RANGE) {
            theNet.GetNode(node)->Value()->SetEvidence(0);
        }
    }
    theNet.UpdateBeliefs();
    int hypothesis = theNet.FindNode("V_0");
    theNet.GetNode(hypothesis)->Value();
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(hypothesis)->Value());
    DSL_idArray *theNames = theNet.GetNode(hypothesis)->Definition()->GetOutcomesNames();
    theCoordinates[0] = theNames->FindPosition("YES");
    theCoordinates.GoToCurrentPosition();
    double hypothesisProbability = theCoordinates.UncheckedValue();
    LOG(LWARNING) << "Hypothesis probability: " << hypothesisProbability;
    hypothesesProbabilities.push_back(hypothesisProbability);
    out_probabilities.write(hypothesesProbabilities);
}


}//: namespace Network
}//: namespace Processors
