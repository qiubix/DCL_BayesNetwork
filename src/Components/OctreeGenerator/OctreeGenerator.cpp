/*!
 * \file OctreeGenerator.cpp
 * \brief
 */

//#include <memory>
#include <string>
//#include <iostream>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include "OctreeGenerator.hpp"

#include "Logger.hpp"

namespace Generators {
namespace Network {

OctreeGenerator::OctreeGenerator(const std::string & name) : Base::Component(name) {
  LOG(LTRACE)<<"Hello OctreeGenerator\n";
  octree = NULL;
}

OctreeGenerator::~OctreeGenerator() {
  LOG(LTRACE)<<"Good bye OctreeGenerator\n";
}

void OctreeGenerator::prepareInterface() {
  LOG(LTRACE) << "OctreeGenerator::prepareInterface\n";
  registerStream("out_octree", &out_octree);
}

//void OctreeGenerator::setPointCloud(pcl::PointCloud<PointXYZSIFT>::Ptr cloud) {
//  this -> cloud =  cloud;
//}

Octree* OctreeGenerator::getOctree() {
  return octree;
}

//pcl::PointCloud<PointXYZSIFT>::Ptr OctreeGenerator::getPointCloud() {
//  return cloud;
//}

void OctreeGenerator::buildOctree() {
//  double voxelSize = 0.01f;
//  octree = new OctreeWithSIFT(voxelSize);

  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = getPointCloud();
  octree = new Octree(cloud);
  octree->init();

//  octree -> setInputCloud(cloud);
//  octree -> defineBoundingBox();
//  octree -> addPointsFromInputCloud();
}

pcl::PointCloud<PointXYZSIFT>::Ptr OctreeGenerator::getPointCloud() const {
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud(new pcl::PointCloud<PointXYZSIFT>);
  cloud->width = 8;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (int j = 0; j < cloud->width; ++j) {
    cloud->points[j].pointId = j;
    cloud->points[j].x = (float) (j + 0.1);
    cloud->points[j].y = (float) (j + 0.2);
    cloud->points[j].z = (float) (j + 0.3);
    for(int i=0; i<128; i++) {
      cloud->points[j].descriptor[i] = i;
    }
  }

  return cloud;
}

bool OctreeGenerator::onInit() {
  LOG(LTRACE) << "OctreeGenerator::initialize\n";
  return true;
}

bool OctreeGenerator::onFinish() {
  LOG(LTRACE) << "OctreeGenerator::finish\n";
  return true;
}

bool OctreeGenerator::onStop() {
  LOG(LTRACE) << "OctreeGenerator::onStop\n";
  return true;
}

bool OctreeGenerator::onStart() {
  LOG(LTRACE) << "OctreeGenerator::onStart\n";
  buildOctree();
  return true;
}

}//: namespace Network
}//: namespace Generators
