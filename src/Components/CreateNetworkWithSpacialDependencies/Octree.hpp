#ifndef OCTREE_HPP
#define OCTREE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Types/PointXYZSIFT.hpp>

#include "OctreeContainers.hpp"

using namespace pcl::octree;

namespace Processors {
namespace Network {

class Octree {
public:
  Octree(pcl::PointCloud<PointXYZSIFT>::Ptr cloud);
  ~Octree();

  void init();

private:
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
  float voxelSize;
  OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>* octree;

};

}//:Network
}//:Processors

#endif //OCTREE_HPP
