#include "OctreeNode.hpp"


namespace Processors {
namespace Network {

OctreeNode::OctreeNode(pcl::octree::OctreeNode* node) {
  this->node = node;
  switch(node->getNodeType()) {
    case BRANCH_NODE: this->nodeType = OCTREE_BRANCH_NODE; break;
    case LEAF_NODE: this->nodeType = OCTREE_LEAF_NODE; break;
  }
}

OctreeNode::OctreeNode(const OctreeNode& copy) {
  this->node = copy.node;
  this->nodeType = copy.getNodeType();
}

NodeType OctreeNode::getNodeType() const {
  return nodeType;
}

pcl::octree::OctreeNode* OctreeNode::getNodePtr() {
  return node;
}

}//:Network
}//:Processors
