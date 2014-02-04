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
    LOG(LTRACE)<<"Hello SOMSimpleEvaluation\n";
}

SOMSimpleEvaluation::~SOMSimpleEvaluation()
{
    LOG(LTRACE)<<"Good bye SOMSimpleEvaluation\n";
}

void SOMSimpleEvaluation::prepareInterface()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::prepareInterface\n";
    h_onModels.setup(this, &SOMSimpleEvaluation::onModels);
    registerHandler("onModels", &h_onModels);
    
    registerStream("in_models", &in_models);
    registerStream("in_jointMultiplicity", &in_jointMultiplicity);
    addDependency("onModels", &in_models);
    addDependency("onModels", &in_jointMultiplicity);
}

bool SOMSimpleEvaluation::onInit()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::initialize\n";
    return true;
}

bool SOMSimpleEvaluation::onFinish()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::finish\n";
    return true;
}

bool SOMSimpleEvaluation::onStart()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::onStart\n";
    return true;
}

bool SOMSimpleEvaluation::onStop()
{
    LOG(LTRACE) << "SOMSimpleEvaluation::onStop\n";
    return true;
}

void SOMSimpleEvaluation::onModels()
{
    models = in_models.read();
    jointMultiplicityVector = in_jointMultiplicity.read();
}

void SOMSimpleEvaluation::onInstance()
{
    
}


}//: namespace Network
}//: namespace Processors
