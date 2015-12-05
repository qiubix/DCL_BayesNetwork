/*!
 * \file OctreeBuilder.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <iostream>

#include "OctreeBuilder.hpp"

#include "Logger.hpp"

namespace Processors {
namespace Network {

OctreeBuilder::OctreeBuilder(const std::string & name) : Base::Component(name)
{
    LOG(LTRACE)<<"Hello OctreeBuilder\n";
}

OctreeBuilder::~OctreeBuilder()
{
    LOG(LTRACE)<<"Good bye OctreeBuilder\n";
}

void OctreeBuilder::prepareInterface()
{
    LOG(LTRACE) << "OctreeBuilder::prepareInterface\n";
}

bool OctreeBuilder::onInit()
{
    LOG(LTRACE) << "OctreeBuilder::initialize\n";

    return true;
}

bool OctreeBuilder::onFinish()
{
    LOG(LTRACE) << "OctreeBuilder::finish\n";

    return true;
}

bool OctreeBuilder::onStop()
{
    LOG(LTRACE) << "OctreeBuilder::onStop\n";

    return true;
}

bool OctreeBuilder::onStart()
{
    LOG(LTRACE) << "OctreeBuilder::onStart\n";

    return true;
}

}//: namespace Network
}//: namespace Processors
