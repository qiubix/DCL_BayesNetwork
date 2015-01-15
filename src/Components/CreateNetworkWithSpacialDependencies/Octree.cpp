#include "Octree.hpp"

using namespace pcl::octree;

namespace Processors {
namespace Network {

Octree::Octree() {
	voxelSize = 0.01f;
	octree = new OctreePointCloud<PointXYZSIFT, OctreeContainerPointIndicesWithId, OctreeContainerEmptyWithId> (voxelSize);
}

Octree::~Octree() {
  delete octree;
}

void Octree::init() {

}

}//:Network
}//:Processors
