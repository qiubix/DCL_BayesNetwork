#ifndef OCTREE_BRANCH_NODE_HPP
#define OCTREE_BRANCH_NODE_HPP

#include "OctreeNode.hpp"

namespace Processors {
namespace Network {

class OctreeBranchNode : public OctreeNode {
  public:
    OctreeBranchNode(pcl::octree::OctreeNode* node);
    //OctreeBranchNode(const OctreeBranchNode& copy);
    ~OctreeBranchNode() {}
  private:
    pcl::octree::OctreeBranchNode<OctreeContainerEmptyWithId>* branchNode;
};

}//:Network
}//:Processors


#endif //OCTREE_BRANCH_NODE_HPP
