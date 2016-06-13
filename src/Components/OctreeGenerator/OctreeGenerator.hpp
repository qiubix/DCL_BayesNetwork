/*!
 * \file OctreeGenerator.hpp
 * \brief Component generating pcl::octree
 */

#pragma once
#ifndef PCL_OCTREE_GENERATOR_HPP_
#define PCL_OCTREE_GENERATOR_HPP_

//#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
//#include "Property.hpp"

//#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>

#include "Types/PointXYZSIFT.hpp"
#include <Types/OctreeContainers.hpp>
#include <Types/Octree.hpp>

typedef pcl::octree::OctreePointCloud<PointXYZSIFT,
    Processors::Network::OctreeContainerPointIndicesWithId,
    Processors::Network::OctreeContainerEmptyWithId> OctreeWithSIFT;

using Processors::Network::Octree;

namespace Generators {
namespace Network {

/*!
 * \class OctreeGenerator
 * \brief Component for building octree from point cloud
 * \author Karol Katerżawa
 */
class OctreeGenerator: public Base::Component
{
public:
  OctreeGenerator(const std::string & name = "OctreeGenerator");
  virtual ~OctreeGenerator();

  void prepareInterface();
//  void setPointCloud(pcl::PointCloud<PointXYZSIFT>::Ptr cloud);
//  pcl::PointCloud<PointXYZSIFT>::Ptr getPointCloud();

  void buildOctree();
  Octree* getOctree();

  bool onStart();
  bool onInit();
  bool onFinish();
  bool onStop();

protected:

  /// Output data stream
  Base::DataStreamOut< pcl::PointCloud<PointXYZSIFT>::Ptr > out_octree;

private:
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
  Octree* octree;
  pcl::PointCloud<PointXYZSIFT>::Ptr getPointCloud() const;
};

}//: namespace Network
}//: namespace Generators


/*
 * Register processor component.
 */
REGISTER_COMPONENT("OctreeGenerator", Generators::Network::OctreeGenerator)

#endif /* PCL_OCTREE_GENERATOR_HPP_ */

