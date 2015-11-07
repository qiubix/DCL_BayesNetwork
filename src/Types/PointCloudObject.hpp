#ifndef POINTCLOUDOBJECT_HPP_
#define POINTCLOUDOBJECT_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/AbstractObject.hpp> 

//namespace Types {

/*!
 * \class PointCloudObject
 * \brief A 3D objects modelled as a cloud of points.
 * It consists of It consists of: name and object point cloud.
 */
class PointCloudObject : virtual public AbstractObject
{
	public:
	/// Cloud of points constituting the object model.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;
};


//} //: namespace Types

#endif /* POINTCLOUDOBJECT_HPP_ */
