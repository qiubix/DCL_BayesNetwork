#include "DepthFirstIterator.hpp"
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

namespace Processors {
namespace Network {

typedef pcl::octree::OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::DepthFirstIterator OctreeIterator;

DepthFirstIterator::DepthFirstIterator(OctreeIterator it) {
  this->it = it;
}

DepthFirstIterator DepthFirstIterator::operator++() {
  DepthFirstIterator dfIt = *this;
  it++;
  return dfIt;
}

DepthFirstIterator DepthFirstIterator::operator++(int foo) {
  it++;
  return *this;
}

pcl::octree::OctreeNode* DepthFirstIterator::operator*() {
  return it.getCurrentOctreeNode();
}

OctreeIterator* DepthFirstIterator::operator->() {
  return &it;
}

bool DepthFirstIterator::operator==(const DepthFirstIterator& reference) {
  return it == reference.it;
}

bool DepthFirstIterator::operator!=(const DepthFirstIterator& reference) {
  return it != reference.it;
}

}//:Network
}//:Processors
