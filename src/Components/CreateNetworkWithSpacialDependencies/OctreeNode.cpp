#include "OctreeNode.hpp"


namespace Processors {
namespace Network {

OctreeNode::OctreeNode() {
}

OctreeNode::OctreeNode(pcl::octree::OctreeNode* node) {
  this->node = node;
  //this->nodeType = node->getNodeType();
  switch(node->getNodeType()) {
    case BRANCH_NODE: this->nodeType = OCTREE_BRANCH_NODE; break;
    case LEAF_NODE: this->nodeType = OCTREE_LEAF_NODE; break;
  }
}

OctreeNode::OctreeNode(const OctreeNode& copy) {
  this->node = copy.node;
  //this->nodeType = node->getNodeType();
  switch(node->getNodeType()) {
    case BRANCH_NODE: this->nodeType = OCTREE_BRANCH_NODE; break;
    case LEAF_NODE: this->nodeType = OCTREE_LEAF_NODE; break;
  }
}

NodeType OctreeNode::getNodeType() {
  return nodeType;
}

}//:Network
}//:Processors
