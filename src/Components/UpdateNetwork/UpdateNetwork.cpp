/*!
 * \file UpdateNetwork.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <iostream>

#include "UpdateNetwork.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

UpdateNetwork::UpdateNetwork(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello UpdateNetwork\n";
}

UpdateNetwork::~UpdateNetwork()
{
    LOG(LTRACE)<<"Good bye UpdateNetwork\n";
}

void UpdateNetwork::prepareInterface()
{
    LOG(LTRACE) << "UpdateNetwork::prepareInterface\n";
    h_onNetwork.setup(this, &UpdateNetwork::onNetwork);
    registerHandler("onNetwork", &h_onNetwork);
//    h_onObservation.setup(this, &UpdateNetwork::onObservation);
//    registerHandler("onObservation", &h_onObservation);

    registerStream("in_network", &in_network);
    addDependency("onNetwork", &in_network);

}

bool UpdateNetwork::onInit()
{
    LOG(LTRACE) << "UpdateNetwork::initialize\n";

    return true;
}

bool UpdateNetwork::onFinish()
{
    LOG(LTRACE) << "UpdateNetwork::finish\n";

    return true;
}

bool UpdateNetwork::onStop()
{
    LOG(LTRACE) << "UpdateNetwork::onStop\n";

    return true;
}

bool UpdateNetwork::onStart()
{
    LOG(LTRACE) << "UpdateNetwork::onStart\n";

    return true;
}

void UpdateNetwork::onNetwork()
{
    LOG(LTRACE) << "UpdateNetwork::onNetwork\n";
}

void UpdateNetwork::onObservation()
{
    LOG(LTRACE) << "UpdateNetwork::onObservation\n";
}

void UpdateNetwork::changeNodeCPT(const string nodeName, vector<double> probabilities)
{
    LOG(LTRACE) << "UpdateNetwork::changeNodeProbabilities";
    int node = theNet.FindNode(nodeName.c_str());
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(node)->Definition());
    std::vector<double>::iterator it = probabilities.begin();
    do {
        theCoordinates.UncheckedValue() = *it;
        ++it;
    } while(theCoordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

void UpdateNetwork::observeNode(string observedNode, int observedState)
{
    LOG(LTRACE) << "UpdateNetwork::observeNode";
    int node = theNet.FindNode(observedNode.c_str());
    theNet.GetNode(node)->Value()->SetEvidence(observedState);
}

void UpdateNetwork::clearEvidence()
{
    theNet.ClearAllEvidence();
}

}//: namespace Network
}//: namespace Processors
