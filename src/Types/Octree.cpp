#include "Octree.hpp"
#include "DepthFirstIterator.hpp"
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

using namespace pcl::octree;

namespace Processors {
namespace Network {

Octree::Octree(pcl::PointCloud<PointXYZSIFT>::Ptr cloud) {
  this -> cloud = cloud;
  voxelSize = 0.01f;
  octree = new OctreeWithSIFT (voxelSize);
}

Octree::~Octree() {
  //Delete octree data structure (pushes allocated nodes to memory pool!).
  octree->deleteTree();
  //FIXME: should this be freed?
//  delete octree;
}

//TODO: move this to constructor
void Octree::init() {
  // Set input cloud.
  octree->setInputCloud(cloud);
  // Calculate bounding box of input cloud.
  octree->defineBoundingBox();
  // Add points from input cloud to octree.
  octree->addPointsFromInputCloud();
}

int Octree::getNumberOfPoints() {
  return octree -> getLeafCount();
}

bool Octree::empty() {
  return cloud -> empty();
}

PointXYZSIFT Octree::getPoint(unsigned int id) {
  return cloud -> at(id);
}

DepthFirstIterator Octree::depthBegin() {
  return octree -> depth_begin();
}

DepthFirstIterator Octree::depthEnd() {
  return octree -> depth_end();
}

Octree::OctreeWithSIFT Octree::getOctreeWithSIFT() {
  return *octree;
}

pcl::PointCloud<PointXYZSIFT>::Ptr Octree::getInputCloud() {
  return cloud;
}

}//:Network
}//:Processors
