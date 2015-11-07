#ifndef SIFTOBJECTMODEL_HPP_
#define SIFTOBJECTMODEL_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointCloudObject.hpp> 
#include <Types/PointXYZSIFT.hpp> 
#include <Types/PointCloudNormalObject.hpp>

//namespace Types {

/*!
 * \class SIFTObjectModel
 * \brief Model of 3D object.
 * It consists of: object point cloud, SIFT cloud, mean number of viewpoint features.
 */
class SIFTObjectModel : public PointCloudObject, public PointCloudNormalObject
{
	public:
	/// Mean number of viewpoint features
	int mean_viewpoint_features_number;

	/// Cloud of SIFT - features extracted from RGB image and transformed from image into Cartesian space.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;
};


//} //: namespace Types

#endif /* SIFTOBJECTMODEL_HPP_ */
