/*!
 * \file OctreeBuilder.hpp
 * \brief Component for building octree from point cloud
 */

#ifndef OCTREE_BUILDER_HPP_
#define OCTREE_BUILDER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../../Types/PointXYZSIFT.hpp"
#include "../../Types/Octree.hpp"

namespace Processors {
namespace Network {

/*!
 * \class OctreeBuilder
 * \brief Component for building octree from point cloud
 * \author Karol Katerżawa
 */
class OctreeBuilder: public Base::Component
{
public:
  OctreeBuilder(const std::string & name = "OctreeBuilder");
  virtual ~OctreeBuilder();

  void prepareInterface();
  void setPointCloud(pcl::PointCloud<PointXYZSIFT>::Ptr cloud);
  pcl::PointCloud<PointXYZSIFT>::Ptr getPointCloud();

  void buildOctree();
  Octree getOctree();

protected:

  /// Input data stream
  Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud;

  /// Output data stream
  Base::DataStreamOut< Octree* > out_octree;

  bool onInit();
  bool onFinish();
  bool onStart();
  bool onStop();

  /*!
   * Event handler function.
   */
  void onNewCloud();

private:
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
  Octree* octree;
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("OctreeBuilder", Processors::Network::OctreeBuilder)

#endif /* OCTREE_BUILDER_HPP_ */

