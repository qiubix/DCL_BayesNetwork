#include "OctreeBranchNode.hpp"

namespace Processors {
namespace Network {

OctreeBranchNode::OctreeBranchNode(pcl::octree::OctreeNode* node) : OctreeNode(node) {
  branchNode = static_cast<pcl::octree::OctreeBranchNode<OctreeContainerEmptyWithId>* >(node);
}

//OctreeBranchNode::OctreeBranchNode(const OctreeBranchNode& copy) {
//}

}//:Network
}//:Processors
