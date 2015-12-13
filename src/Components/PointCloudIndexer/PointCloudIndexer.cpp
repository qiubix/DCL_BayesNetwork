/*!
 * \file PointCloudIndexer.cpp
 * \brief
 */

#include "PointCloudIndexer.hpp"

#include "Logger.hpp"

namespace Processors {
namespace Network {

PointCloudIndexer::PointCloudIndexer(const std::string & name) : Base::Component(name) {
  LOG(LTRACE)<<"Hello PointCloudIndexer\n";
}

PointCloudIndexer::~PointCloudIndexer() {
  LOG(LTRACE)<<"Good bye PointCloudIndexer\n";
}

void PointCloudIndexer::prepareInterface() {
  LOG(LTRACE) << "PointCloudIndexer::prepareInterface\n";
}

void PointCloudIndexer::setPointCloud(pcl::PointCloud<PointXYZSIFT>::Ptr cloud) {
  this -> cloud =  cloud;
}

pcl::PointCloud<PointXYZSIFT>::Ptr PointCloudIndexer::getPointCloud() {
  return cloud;
}

void PointCloudIndexer::indexPoints() {
  for (int i=0; i<cloud->points.size(); ++i) {
    cloud -> points[i].pointId = i;
  }
}

bool PointCloudIndexer::onInit() {
  LOG(LTRACE) << "PointCloudIndexer::initialize\n";
  return true;
}

bool PointCloudIndexer::onFinish() {
  LOG(LTRACE) << "PointCloudIndexer::finish\n";
  return true;
}

bool PointCloudIndexer::onStop() {
  LOG(LTRACE) << "PointCloudIndexer::onStop\n";
  return true;
}

bool PointCloudIndexer::onStart() {
  LOG(LTRACE) << "PointCloudIndexer::onStart\n";
  return true;
}

}//: namespace Network
}//: namespace Processors
