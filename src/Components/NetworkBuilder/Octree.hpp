#ifndef OCTREE_HPP
#define OCTREE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../../Types/PointXYZSIFT.hpp"

#include "OctreeContainers.hpp"

using namespace pcl::octree;

namespace Processors {
namespace Network {

class Octree {
public:
  Octree(pcl::PointCloud<PointXYZSIFT>::Ptr cloud);
  ~Octree();

  void init();

  class DepthFirstIterator {
  public:
    typedef OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>::DepthFirstIterator OctreeIterator;
    DepthFirstIterator(OctreeIterator it) {
      this->it = it;
    }
    DepthFirstIterator operator++() {
      DepthFirstIterator dfIt = *this;
      it++;
      return dfIt;
    }
    DepthFirstIterator operator++(int foo) {
      it++;
      return *this;
    }
    pcl::octree::OctreeNode* operator*() {
      return it.getCurrentOctreeNode();
    }
    OctreeIterator* operator->() {
      return &it;
    }
    bool operator==(const DepthFirstIterator& reference) {
      return it == reference.it;
    }
    bool operator!=(const DepthFirstIterator& reference) {
      return it != reference.it;
    }
  private:
    OctreeIterator it;
  };

  DepthFirstIterator depthBegin();
  DepthFirstIterator depthEnd();

private:
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
  float voxelSize;
  OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>* octree;

};

}//:Network
}//:Processors

#endif //OCTREE_HPP
