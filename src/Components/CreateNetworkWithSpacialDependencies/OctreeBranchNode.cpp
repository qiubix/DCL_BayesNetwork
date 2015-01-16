#include "OctreeBranchNode.hpp"

namespace Processors {
namespace Network {

OctreeBranchNode::OctreeBranchNode(pcl::octree::OctreeNode* node) : OctreeNode(node) {
  branchNode = static_cast<pcl::octree::OctreeBranchNode<OctreeContainerEmptyWithId>* >(node);
}

OctreeBranchNode::OctreeBranchNode(OctreeNode octreeNode) : OctreeNode(octreeNode) {
  branchNode = static_cast<pcl::octree::OctreeBranchNode<OctreeContainerEmptyWithId>* >(this->node);
}

//OctreeBranchNode::OctreeBranchNode(const OctreeBranchNode& copy) {
//}

int OctreeBranchNode::getId() {
  return branchNode->getContainer().getNodeId();
}

void OctreeBranchNode::setId(int id) {
  return branchNode->getContainer().setNodeId(id);
}

}//:Network
}//:Processors
