#pragma once
#ifndef BAYESNETWORK_OCTREEDEPTHFIRSTITERATOR_HPP
#define BAYESNETWORK_OCTREEDEPTHFIRSTITERATOR_HPP

#include "PointXYZSIFT.hpp"
#include "OctreeContainers.hpp"
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_nodes.h>

namespace Processors {
namespace Network {

class OctreeNode;

class DepthFirstIterator {
public:
  typedef pcl::octree::OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::DepthFirstIterator OctreeIterator;

  DepthFirstIterator(OctreeIterator it);

  DepthFirstIterator operator++();
  DepthFirstIterator operator++(int foo);

//  pcl::octree::OctreeNode* operator*();

  OctreeNode getCurrentNode();
  OctreeIterator* operator->();

  bool operator==(const DepthFirstIterator& reference);
  bool operator!=(const DepthFirstIterator& reference);

private:
  OctreeIterator it;
};

}//:Network
}//:Processors

#endif //BAYESNETWORK_OCTREEDEPTHFIRSTITERATOR_HPP
