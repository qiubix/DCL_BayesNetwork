#include "OctreeLeafNode.hpp"

namespace Processors {
namespace Network {

OctreeLeafNode::OctreeLeafNode(pcl::octree::OctreeNode* node) : OctreeNode(node) {
  leafNode = static_cast<pcl::octree::OctreeLeafNode<OctreeContainerEmptyWithId>* >(node);
}

OctreeLeafNode::OctreeLeafNode(OctreeNode octreeNode) : OctreeNode(octreeNode) {
  leafNode = static_cast<pcl::octree::OctreeLeafNode<OctreeContainerEmptyWithId>* >(this->node);
}

//OctreeLeafNode::OctreeLeafNode(const OctreeLeafNode& copy) {
//}

int OctreeLeafNode::getId() {
  return leafNode->getContainer().getNodeId();
}

void OctreeLeafNode::setId(int id) {
  return leafNode->getContainer().setNodeId(id);
}

int OctreeLeafNode::getNumberOfChildren() {
  return leafNode->getContainer().getSize();
}

std::vector<int> OctreeLeafNode::getPointIndices() {
	std::vector<int> point_indices;
	leafNode->getContainer().getPointIndices(point_indices);
	return point_indices;
}

}//:Network
}//:Processors

