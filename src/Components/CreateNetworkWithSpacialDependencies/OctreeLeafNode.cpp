#include "OctreeLeafNode.hpp"

namespace Processors {
namespace Network {

OctreeLeafNode::OctreeLeafNode(pcl::octree::OctreeNode* node) : OctreeNode(node) {
  branchNode = static_cast<pcl::octree::OctreeLeafNode<OctreeContainerEmptyWithId>* >(node);
}

OctreeLeafNode::OctreeLeafNode(OctreeNode octreeNode) : OctreeNode(octreeNode) {
  branchNode = static_cast<pcl::octree::OctreeLeafNode<OctreeContainerEmptyWithId>* >(this->node);
}

//OctreeLeafNode::OctreeLeafNode(const OctreeLeafNode& copy) {
//}

int OctreeLeafNode::getId() {
  return branchNode->getContainer().getNodeId();
}

void OctreeBranchNode::setId(int id) {
  return branchNode->getContainer().setNodeId(id);
}

}//:Network
}//:Processors

