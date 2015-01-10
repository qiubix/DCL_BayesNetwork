/*!
 * \file MockNetworkBuilder.cpp
 * \brief
 */

#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "MockNetworkBuilder.hpp"

#include "Logger.hpp"
#include "Common/Timer.hpp"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace Processors {
namespace Network {

MockNetworkBuilder::MockNetworkBuilder(const std::string & name) : Base::Component(name)
{
  LOG(LTRACE)<<"Hello MockNetworkBuilder\n";
  branchNodeCount = 0;
  leafNodeCount = 0;
  maxLeafContainerSize = 0;
  nextId = 0;
  numberOfVoxels = 0;
}

MockNetworkBuilder::~MockNetworkBuilder()
{
  LOG(LTRACE)<<"Good bye MockNetworkBuilder\n";
}

void MockNetworkBuilder::prepareInterface()
{
	LOG(LTRACE) << "MockNetworkBuilder::prepareInterface\n";

	// Register data streams.
	//	registerStream("in_cloud", &in_cloud_xyz);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_jointMultiplicity", &in_jointMultiplicity);
	// Register handlers
	h_onNewModel.setup(boost::bind(&MockNetworkBuilder::onNewModel, this));
	h_onJointMultiplicity.setup(boost::bind(&MockNetworkBuilder::onJointMultiplicity, this));
	registerHandler("onNewModel", &h_onNewModel);
	registerHandler("onJointMultiplicity", &h_onJointMultiplicity);
	addDependency("onNewModel", &in_cloud_xyzsift);
	addDependency("onJointMultiplicity", &in_jointMultiplicity);

	//registerStream("out_network", &out_network);
	registerStream("out_networks", &out_networks);
}

bool MockNetworkBuilder::onInit()
{
  LOG(LTRACE) << "MockNetworkBuilder::initialize\n";
  return true;
}

bool MockNetworkBuilder::onFinish()
{
  LOG(LTRACE) << "MockNetworkBuilder::finish\n";
  return true;
}

bool MockNetworkBuilder::onStop()
{
  LOG(LTRACE) << "MockNetworkBuilder::onStop\n";
  return true;
}

void MockNetworkBuilder::onNewModel()
{
  LOG(LTRACE) << "On new model";
  pcl::PointCloud<PointXYZSIFT>::Ptr newCloud = in_cloud_xyzsift.read();
  cloudQueue.push(newCloud);
}

void MockNetworkBuilder::onJointMultiplicity()
{
  LOG(LTRACE) << "On joint multiplicity";
  jointMultiplicityVector = in_jointMultiplicity.read();
  if (cloudQueue.size() > 0) {
    buildNetwork();
  }
}

bool MockNetworkBuilder::onStart()
{
  LOG(LTRACE) << "MockNetworkBuilder::onStart\n";
  return true;
}

void MockNetworkBuilder::buildNetwork() {
	LOG(LDEBUG) << "MockNetworkBuilder::buildNetwork";

  if(network.getNumberOfNodes() != 0) {
    return;
  }

  LOG(LDEBUG) << "Size of cloudQueue: " << cloudQueue.size();
	// Read from queue
	cloud = cloudQueue.top();
  cloudQueue.pop();
//  jointMultiplicityVector = in_jointMultiplicity.read();

  exportNetwork();
}
void MockNetworkBuilder::exportNetwork()
{
	LOG(LDEBUG) << "before writing network to file";
  network.exportNetworkToFile();
	LOG(LDEBUG) << "after writing network to file";
  std::vector<DSL_network> networks;
  networks.push_back(network.getNetwork());
  out_networks.write(networks);
	//out_network.write(network.getNetwork());
}

}//: namespace Network
}//: namespace Processors
