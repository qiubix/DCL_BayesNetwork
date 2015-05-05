#ifndef OCTREE_NODE_HPP
#define OCTREE_NODE_HPP

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../../Types/PointXYZSIFT.hpp"

#include "OctreeContainers.hpp"

namespace Processors {
namespace Network {

enum NodeType {
  OCTREE_BRANCH_NODE,
  OCTREE_LEAF_NODE
};

class OctreeNode {
public:
  OctreeNode(pcl::octree::OctreeNode* node);
  OctreeNode(const OctreeNode& copy);
  virtual ~OctreeNode() {}

  //getters
  NodeType getNodeType() const;
  pcl::octree::OctreeNode* getNodePtr();
  //FIXME: it should have throw exception, or somehow calculate number of children
  // maybe this class should be abstract, so this method would be pure virtual
  virtual int getNumberOfChildren() {}
  virtual int getId() {}
  virtual void setId(int id) {}

protected:
  pcl::octree::OctreeNode* node;
  NodeType nodeType;
};

}//:Network
}//:Processors

#endif //OCTREE_NODE_HPP
