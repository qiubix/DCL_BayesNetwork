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
    }
}

void SOMSimpleEvaluation::evaluate()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::evaluate";
}


}//: namespace Network
}//: namespace Processors
