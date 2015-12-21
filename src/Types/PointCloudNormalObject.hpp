#ifndef POINTCLOUDNORMALOBJECT_HPP_
#define POINTCLOUDNORMALOBJECT_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/AbstractObject.hpp> 

//namespace Types {

/*!
 * \class PointCloudObject
 * \brief A 3D objects modelled as a cloud of points.
 * It consists of It consists of: name and object point cloud.
 */
class PointCloudNormalObject : virtual public AbstractObject
{
	public:
	/// Cloud of points constituting the object model.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgb_normals;
};


//} //: namespace Types

#endif /* POINTCLOUDNORMALOBJECT_HPP_ */
