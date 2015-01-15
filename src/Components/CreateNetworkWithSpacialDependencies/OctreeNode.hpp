#ifndef OCTREE_NODE_HPP
#define OCTREE_NODE_HPP

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Types/PointXYZSIFT.hpp>

#include "OctreeContainers.hpp"

namespace Processors {
namespace Network {

class OctreeNode {
public:
  OctreeNode();
  OctreeNode(pcl::octree::OctreeNode* node);
  ~OctreeNode() {}

  //TODO: copy constructor

private:
  pcl::octree::OctreeNode* node;
};

}//:Network
}//:Processors

#endif //OCTREE_NODE_HPP
