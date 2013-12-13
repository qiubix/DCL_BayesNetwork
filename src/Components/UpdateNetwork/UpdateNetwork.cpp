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
    //h_onStep.setup(this, &UpdateNetwork::onStep);
    //registerHandler("onStep", &h_onStep);

    registerStream("in_network", &in_network);

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

bool UpdateNetwork::onStep()
{
    LOG(LTRACE) << "UpdateNetwork::onStep\n";

    return true;
}


void UpdateNetwork::onNewImage() {
    LOG(LTRACE) << "UpdateNetwork::onNewImage\n";
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
