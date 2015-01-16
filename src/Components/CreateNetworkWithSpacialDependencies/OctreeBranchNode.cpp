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

bool OctreeBranchNode::hasOnlyOneChild() {
  int childrenCounter = 0;
  for (unsigned child_idx = 0; child_idx < 8; ++child_idx) {
    if (branchNode -> hasChild(child_idx))
      ++childrenCounter;
  }
  if (childrenCounter == 1)
    return true;
  else
    return false;
}

bool OctreeBranchNode::nextNodeIsAlsoBranchNode() {
  pcl::octree::OctreeNode* childNode;
  unsigned char index;
  for (index = 0; index < 8; ++index) {
    if (branchNode -> hasChild(index))
			childNode = branchNode -> getChildPtr(index);
  }
  if (childNode -> getNodeType() == BRANCH_NODE)
    return true;
  else
    return false;
}

int OctreeBranchNode::getNumberOfChildren() {
  unsigned char index;
  int childrenCounter = 0;
  for (index = 0; index < 8; ++index) {
    if (branchNode -> hasChild(index))
			++childrenCounter;
  }
  return childrenCounter;
}

}//:Network
}//:Processors
