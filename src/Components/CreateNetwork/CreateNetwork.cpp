/*!
 * \file CreateNetwork.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <iostream>

#include "CreateNetwork.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

CreateNetwork::CreateNetwork(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello CreateNetwork\n";
}

CreateNetwork::~CreateNetwork()
{
    LOG(LTRACE)<<"Good bye CreateNetwork\n";
}

void CreateNetwork::prepareInterface()
{
    LOG(LTRACE) << "CreateNetwork::prepareInterface\n";
}

bool CreateNetwork::onInit()
{
    LOG(LTRACE) << "CreateNetwork::initialize\n";

    return true;
}

bool CreateNetwork::onFinish()
{
    LOG(LTRACE) << "CreateNetwork::finish\n";

    return true;
}

bool CreateNetwork::onStop()
{
    LOG(LTRACE) << "CreateNetwork::onStop\n";

    return true;
}

bool CreateNetwork::onStart()
{
    LOG(LTRACE) << "CreateNetwork::onStart\n";

    return true;
}

bool CreateNetwork::onStep()
{
    LOG(LTRACE) << "CreateNetwork::onStep\n";

    return true;
}


void CreateNetwork::onNewImage() {
    LOG(LTRACE) << "CreateNetwork::onNewImage\n";
}

void CreateNetwork::initNetwork()
{
    //TODO: getting data from input stream and building network on the basics of this data
}

void CreateNetwork::loadNetwork()
{
    //TODO: loading previously build network from file
    int result = -1;
    //result = theNet.ReadFile("/home/qiubix/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    //result = theNet.ReadFile("/home/kkaterza/DCL/BayesNetwork/in_network.xdsl", DSL_XDSL_FORMAT);
    LOG(LWARNING) << "Reading network file: " << result;
}

void CreateNetwork::setBaseNetworkCPTs()
{
    /*
     * TODO: set base CPTs of all nodes in the network
     * using setBaseFeaturesCPTs();
     * using setBaseHypothesesCPTs();
     */
}

void CreateNetwork::setBaseFeaturesCPTs()
{
    /*
     * TODO: set base CPTs of Feature nodes, based on JointMultiplicityVector
     * P(Fi) = ki / sum_j(kj)
     * probability (a'priori) of feature appearence
     */
}

void CreateNetwork::setBaseHypothesesCPTs()
{
    /*
     * TODO: set base CPTs of Hypotheses nodes, based on JointMultiplicityVector
     * P(Hi|Fk, Fk+1, ..., Fn) = P(Hi|Fk)*P(Hi|Fk+1)*...*P(Hi|Fn)
     * probability of hypothesis acceptance
     */
}

void CreateNetwork::addNode(const std::string name, const std::vector<std::string> & outcomesNames, const std::vector<std::string> & parentsNames)
{
    int newNode = theNet.AddNode(DSL_CPT, name.c_str());
    DSL_idArray outcomes;
    for (int i=0; i<outcomesNames.size(); i++) {
        outcomes.Add(outcomesNames[i].c_str());
    }
    theNet.GetNode(newNode)->Definition()->SetNumberOfOutcomes(outcomes);

    int nextParent;
    for (int i=0; i<parentsNames.size(); i++) {
        nextParent = theNet.FindNode(parentsNames[i].c_str());
        theNet.AddArc(nextParent, newNode);
    }
}

void CreateNetwork::setNodeCPT(const string name, vector<double> probabilities)
{
    int node = theNet.FindNode(name.c_str());
    DSL_sysCoordinates theCoordinates(*theNet.GetNode(node)->Definition());

    std::vector<double>::iterator it = probabilities.begin();
    do {
        theCoordinates.UncheckedValue() = *it;
        ++it;
    } while(theCoordinates.Next() != DSL_OUT_OF_RANGE || it != probabilities.end());
}

void CreateNetwork::exportNetwork()
{
    //TODO: exporting network to output datastreams and to file
}

}//: namespace Network
}//: namespace Processors
