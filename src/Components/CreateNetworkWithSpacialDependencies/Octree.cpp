#include "Octree.hpp"

using namespace pcl::octree;

namespace Processors {
namespace Network {

Octree::Octree(pcl::PointCloud<PointXYZSIFT>::Ptr cloud) {
  this->cloud = cloud;
  voxelSize = 0.01f;
  octree = new OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId> (voxelSize);
}

Octree::~Octree() {
  //Delete octree data structure (pushes allocated nodes to memory pool!).
  octree->deleteTree();
  delete octree;
}

void Octree::init() {
  // Set input cloud.
  octree->setInputCloud(cloud);
  // Calculate bounding box of input cloud.
  octree->defineBoundingBox();
  // Add points from input cloud to octree.
  octree->addPointsFromInputCloud();
}

Octree::DepthFirstIterator Octree::depthBegin() {
  return octree->depth_begin();
}

Octree::DepthFirstIterator Octree::depthEnd() {
  return octree->depth_end();
}

}//:Network
}//:Processors
