#ifndef OCTREE_NODE_HPP
#define OCTREE_NODE_HPP

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Types/PointXYZSIFT.hpp>

#include "OctreeContainers.hpp"

namespace Processors {
namespace Network {

enum NodeType {
  OCTREE_BRANCH_NODE,
  OCTREE_LEAF_NODE
};

class OctreeNode {
public:
  OctreeNode();
  OctreeNode(pcl::octree::OctreeNode* node);
  OctreeNode(const OctreeNode& copy);
  ~OctreeNode() {}

  //TODO: copy constructor
  NodeType getNodeType();


private:
  pcl::octree::OctreeNode* node;
  NodeType nodeType;
};

}//:Network
}//:Processors

#endif //OCTREE_NODE_HPP
