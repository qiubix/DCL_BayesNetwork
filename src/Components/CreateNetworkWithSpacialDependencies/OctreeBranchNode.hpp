#ifndef OCTREE_BRANCH_NODE_HPP
#define OCTREE_BRANCH_NODE_HPP

#include "OctreeNode.hpp"

namespace Processors {
namespace Network {

class OctreeBranchNode : public OctreeNode {
  public:
    OctreeBranchNode(pcl::octree::OctreeNode* node);
    OctreeBranchNode(OctreeNode octreeNode);
    //OctreeBranchNode(const OctreeBranchNode& copy);
    ~OctreeBranchNode() {}

    //getters
    int getId();
    void setId(int id);

    bool hasOnlyOneChild();
    bool nextNodeIsAlsoBranchNode();
    int getNumberOfChildren();
  private:
    pcl::octree::OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode;
};

//TODO: figure out cast from OctreeNode

}//:Network
}//:Processors


#endif //OCTREE_BRANCH_NODE_HPP
