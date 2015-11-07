#ifndef SIFTOBJECTINSTANCE_HPP_
#define SIFTOBJECTINSTANCE_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointCloudObject.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointCloudNormalObject.hpp>

//namespace Types {

/*!
 * \class SIFTObjectInstance
 * \brief Instance of 3D object.
 * It consists of: object point cloud (empty as default), model SIFT cloud, instance SIFT cloud, SOM name.
 */
class SIFTObjectModel : public PointCloudObject
{
	public:
	/// Name of the model.
	std::string model_id;

	/// Instance id.
	std::string instance_id;

	/// Reprojection error.
	std::double RPE;



	/// Cloud of SIFTs constituting the whole model
	pcl::PointCloud<PointXYZSIFT>::Ptr model_cloud_xyzsift;

	/// Cloud of SIFTs constituting the instance.
	pcl::PointCloud<PointXYZSIFT>::Ptr instance_cloud_xyzsift;

};


//} //: namespace Types

#endif /* SIFTOBJECTINSTANCE_HPP_ */
