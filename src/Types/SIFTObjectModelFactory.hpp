#ifndef SIFTOBJECTMODELFACTORY_HPP_
#define SIFTOBJECTMODELFACTORY_HPP_

#include <Types/AbstractObjectFactory.hpp> 
#include <Types/SIFTObjectModel.hpp> 


/*!
 * \class SIFTObjectModelFactory
 * \brief Factory responsible for production of SIFT Object Models.
 */
class SIFTObjectModelFactory : public AbstractObjectFactory
{

public:
	SIFTObjectModelFactory(){
		mean_viewpoint_features_number = 0;
	}

	~SIFTObjectModelFactory(){}

	/// Produces and returns a SOM object.
	AbstractObject* produce(){
		SIFTObjectModel *som = new SIFTObjectModel;
		PointCloudNormalObject * somn = new PointCloudNormalObject;
		som->cloud_xyzrgb = cloud_xyzrgb;
		som->cloud_xyzsift = cloud_xyzsift;
		somn->cloud_xyzrgb_normals=cloud_xyzrgb_normals;
		som->name = model_name;
		som->mean_viewpoint_features_number = mean_viewpoint_features_number;
		return som;
	}
	
protected:
	/// Cloud of XYZSIFT points - feature cloud.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;

	/// Cloud of XYZRGB points - object model cloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;

	/// Cloud of XYZRGBNormal points - object model cloud.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgb_normals;

	/// Name of the model.
	std::string model_name;

	/// Mean number of viewpoint features.
	int mean_viewpoint_features_number;
	
};
#endif /* SIFTOBJECTMODELFACTORY_HPP_ */
