#include "OctreeNode.hpp"


namespace Processors {
namespace Network {

OctreeNode::OctreeNode() {
}

OctreeNode::OctreeNode(pcl::octree::OctreeNode* node) {
  this->node = node;
}

}//:Network
}//:Processors
