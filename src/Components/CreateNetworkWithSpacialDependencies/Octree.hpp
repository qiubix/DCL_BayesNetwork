#ifndef OCTREE_HPP
#define OCTREE_HPP

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Types/PointXYZSIFT.hpp>

#include "OctreeContainers.hpp"

using namespace pcl::octree;

namespace Processors {
namespace Network {

class Octree {
public:
  Octree();
  ~Octree();

  void init();

private:
	float voxelSize;
	OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId>* octree;

};

}//:Network
}//:Processors

#endif //OCTREE_HPP
