#include "OctreeBranchNode.hpp"
#include "Logger.hpp"

namespace Processors {
namespace Network {

OctreeBranchNode::OctreeBranchNode(pcl::octree::OctreeNode* node) : OctreeNode(node) {
  branchNode = static_cast<pcl::octree::OctreeBranchNode<OctreeContainerEmptyWithId>* >(node);
}

OctreeBranchNode::OctreeBranchNode(OctreeNode octreeNode) : OctreeNode(octreeNode) {
  branchNode = static_cast<pcl::octree::OctreeBranchNode<OctreeContainerEmptyWithId>* >(this->node);
}

int OctreeBranchNode::getId() {
  return branchNode->getContainer().getNodeId();
}

void OctreeBranchNode::setId(int id) {
  return branchNode->getContainer().setNodeId(id);
}

bool OctreeBranchNode::hasOnlyOneChild() {
  LOG(LTRACE) << "Check whether node has only one child";
  int childrenCounter = 0;
  for (unsigned char child_idx = 0; child_idx < 8; ++child_idx) {
    if (branchNode -> hasChild(child_idx))
      ++childrenCounter;
  }
  return childrenCounter == 1;
}

bool OctreeBranchNode::nextNodeIsAlsoBranchNode() {
  LOG(LTRACE) << "Check whether next node is also a branch node";
  pcl::octree::OctreeNode* childNode;
  unsigned char index;
  for (index = 0; index < 8; ++index) {
    if (branchNode -> hasChild(index))
      childNode = branchNode -> getChildPtr(index);
  }
  //TODO: throw exception when childNode is null
  return childNode -> getNodeType() == BRANCH_NODE;
}

int OctreeBranchNode::getNumberOfChildren() {
  LOG(LTRACE) << "Get number of children";
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
