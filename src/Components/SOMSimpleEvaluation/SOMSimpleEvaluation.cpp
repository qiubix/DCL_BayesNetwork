/*!
 * \file SOMSimpleEvaluation.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "SOMSimpleEvaluation.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

SOMSimpleEvaluation::SOMSimpleEvaluation(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello SOMSimpleEvaluation";
}

SOMSimpleEvaluation::~SOMSimpleEvaluation()
{
    LOG(LTRACE)<<"Good bye SOMSimpleEvaluation";
}

void SOMSimpleEvaluation::prepareInterface()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::prepareInterface";
    h_onModels.setup(this, &SOMSimpleEvaluation::onModels);
    registerHandler("onModels", &h_onModels);
    
    registerStream("in_models", &in_models);
    registerStream("in_jointMultiplicity", &in_jointMultiplicity);
    addDependency("onModels", &in_models);
    addDependency("onModels", &in_jointMultiplicity);
    
    h_onInstance.setup(this, &SOMSimpleEvaluation::onInstance);
    registerHandler("onInstance", &h_onInstance);
    registerStream("in_instance", &in_instance);
    addDependency("onInstance", &in_instance);
    
    registerStream("out_probabilities", &out_probabilities);
}

bool SOMSimpleEvaluation::onInit()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::initialize";
    return true;
}

bool SOMSimpleEvaluation::onFinish()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::finish";
    return true;
}

bool SOMSimpleEvaluation::onStart()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::onStart";
    return true;
}

bool SOMSimpleEvaluation::onStop()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::onStop";
    return true;
}

void SOMSimpleEvaluation::onModels()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::onModels";
    models = in_models.read();
    jointMultiplicityVector = in_jointMultiplicity.read();
}

void SOMSimpleEvaluation::onInstance()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::onInstance";
    if(!models.empty() && !jointMultiplicityVector.empty()) {
		instance = in_instance.read();
        evaluate();
    }
}

void SOMSimpleEvaluation::evaluate()
{
	LOG(LTRACE) << "SOMSimpleEvaluation::evaluate";
	for (unsigned i=0; i<models.size(); ++i) {
		std::map <int, int> model = models[i];
		LOG(LINFO) << "Model nr " << i << ": size = " << model.size();
		int totalModelMultiplicity = 0;
		for( unsigned j=0; j<model.size(); ++j) {
			totalModelMultiplicity += model[j];
		}
		double probability = 0;
		for (unsigned j=0; j<instance.size(); ++j) {
			int featureId = instance[j];
			std::map<int,int>::iterator it = model.find(featureId);
			if (it == model.end()) {
				continue;
			}
			else {
				int multiplicityInModel = it -> second;
				probability += (double) multiplicityInModel/(totalModelMultiplicity * jointMultiplicityVector[featureId]);
			}
		}
		LOG(LINFO) << "Model nr " << i << ": probability = " << probability;
		hypothesesProbabilities.push_back(probability);
	}
}


}//: namespace Network
}//: namespace Processors
