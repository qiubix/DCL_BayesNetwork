/*!
 * \file OctreeBuilder.cpp
 * \brief
 */

//#include <memory>
#include <string>
//#include <iostream>

#include "OctreeBuilder.hpp"

#include "Logger.hpp"

namespace Processors {
namespace Network {

OctreeBuilder::OctreeBuilder(const std::string & name) : Base::Component(name) {
  LOG(LTRACE)<<"Hello OctreeBuilder\n";
}

OctreeBuilder::~OctreeBuilder() {
  LOG(LTRACE)<<"Good bye OctreeBuilder\n";
}

void OctreeBuilder::prepareInterface() {
  LOG(LTRACE) << "OctreeBuilder::prepareInterface\n";
  registerStream("in_cloud", &in_cloud);
  registerStream("out_octree", &out_octree);
  registerHandler("onNewCloud", boost::bind(&OctreeBuilder::onNewCloud, this));
  addDependency("onNewCloud", &in_cloud);
}

void OctreeBuilder::setPointCloud(pcl::PointCloud<PointXYZSIFT>::Ptr cloud) {
  this -> cloud =  cloud;
}

pcl::PointCloud<PointXYZSIFT>::Ptr OctreeBuilder::getPointCloud() {
  return cloud;
}

void OctreeBuilder::buildOctree() {
  octree = new Octree(cloud);
  octree -> init();
}

Octree OctreeBuilder::getOctree() {
  return *octree;
}

bool OctreeBuilder::onInit() {
  LOG(LTRACE) << "OctreeBuilder::initialize\n";
  return true;
}

bool OctreeBuilder::onFinish() {
  LOG(LTRACE) << "OctreeBuilder::finish\n";
  return true;
}

bool OctreeBuilder::onStop() {
  LOG(LTRACE) << "OctreeBuilder::onStop\n";
  return true;
}

bool OctreeBuilder::onStart() {
  LOG(LTRACE) << "OctreeBuilder::onStart\n";
  return true;
}

void OctreeBuilder::onNewCloud() {
  setPointCloud(in_cloud.read());
  buildOctree();
  out_octree.write(octree);
}

}//: namespace Network
}//: namespace Processors
