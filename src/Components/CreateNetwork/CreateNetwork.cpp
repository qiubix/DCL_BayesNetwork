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

void CreateNetwork::addNode(const std::string name, const std::vector<std::string> & outcomes, const std::vector <std::string> & parents)
{
    //TODO: adding single node to the network, setting its properties and connecting it to appropriate parents
    int newNode = theNet.AddNode(DSL_CPT, name);
    DSL_idArray outcomes;


}

void CreateNetwork::exportNetwork()
{
    //TODO: exporting network to output datastreams and to file
}

}//: namespace Network
}//: namespace Processors
