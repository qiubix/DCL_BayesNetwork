#ifndef OCTREE_LEAF_NODE_HPP
#define OCTREE_LEAF_NODE_HPP

#include "OctreeNode.hpp"

namespace Processors {
namespace Network {

class OctreeLeafNode : public OctreeNode {
  public:
    OctreeLeafNode(pcl::octree::OctreeNode* node);
    OctreeLeafNode(OctreeNode octreeNode);
    //OctreeLeafNode(const OctreeLeafNode& copy);
    ~OctreeLeafNode() {}

    //getters
    int getId();
    void setId(int id);
  private:
    pcl::octree::OctreeLeafNode<OctreeContainerEmptyWithId>* branchNode;
};

//TODO: figure out cast from OctreeNode

}//:Network
}//:Processors


#endif //OCTREE_LEAF_NODE_HPP
