#include "OctreeLeafNode.hpp"
#include "Logger.hpp"

namespace Processors {
namespace Network {

OctreeLeafNode::OctreeLeafNode(pcl::octree::OctreeNode* node) : OctreeNode(node) {
  leafNode = static_cast<pcl::octree::OctreeLeafNode<OctreeContainerPointIndicesWithId>* >(node);
}

OctreeLeafNode::OctreeLeafNode(OctreeNode octreeNode) : OctreeNode(octreeNode) {
  leafNode = static_cast<pcl::octree::OctreeLeafNode<OctreeContainerPointIndicesWithId>* >(this->node);
}

int OctreeLeafNode::getId() {
  return leafNode->getContainer().getNodeId();
}

void OctreeLeafNode::setId(int id) {
  leafNode->getContainer().setNodeId(id);
}

int OctreeLeafNode::getNumberOfChildren() {
  return leafNode->getContainer().getSize();
}

std::vector<int> OctreeLeafNode::getPointIndices() {
  LOG(LTRACE) << "Get point indices";
	std::vector<int> point_indices;
	leafNode->getContainer().getPointIndices(point_indices);
	LOG(LTRACE) << "point indices size: " << point_indices.size();
	return point_indices;
}

}//:Network
}//:Processors

