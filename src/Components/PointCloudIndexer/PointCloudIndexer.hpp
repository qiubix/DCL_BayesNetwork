/*!
 * \file PointCloudIndexer.hpp
 * \brief Component for indexing cloud points
 */

#ifndef POINT_CLOUD_INDEXER_HPP_
#define POINT_CLOUD_INDEXER_HPP_

#define CV_NO_BACKWARD_COMPATIBILITY


#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../../Types/PointXYZSIFT.hpp"

namespace Processors {
namespace Network {

/*!
 * \class PointCloudIndexer
 * \brief Component for indexing cloud points
 * \author Karol Katerżawa
 */
class PointCloudIndexer: public Base::Component
{
public:
  PointCloudIndexer(const std::string & name = "PointCloudIndexer");
  virtual ~PointCloudIndexer();

  void prepareInterface();

protected:

  /// Input data stream
  Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud;

  /// Output data stream
  Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > out_cloud;

  bool onInit();
  bool onFinish();
  bool onStart();
  bool onStop();

  /*!
   * Event handler function.
   */

private:
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud;
};

}//: namespace Network
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("PointCloudIndexer", Processors::Network::PointCloudIndexer)

#endif /* POINT_CLOUD_INDEXER_HPP_ */

