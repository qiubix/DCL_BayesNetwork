#pragma once
#ifndef POINTXYZSIFT_HPP_
#define POINTXYZSIFT_HPP_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

//#include <pcl/point_representation.h>

//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/impl/passthrough.hpp>

#include "pcl/impl/instantiate.hpp"

#include <boost/preprocessor.hpp>

//#include "pcl/filters/impl/extract_indices.hpp"
//#include "pcl/filters/impl/voxel_grid.hpp"
//#include "pcl/features/impl/normal_3d.hpp"
//#include "pcl/segmentation/impl/sac_segmentation.hpp"

//#include <pcl/segmentation/extract_clusters.h>
//#include "pcl/segmentation/impl/extract_clusters.hpp"

//#include <pcl/kdtree/impl/kdtree_flann.hpp>

//#include <pcl/search/impl/search.hpp>
//namespace Types {

struct PointXYZSIFT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float descriptor[128];
  int multiplicity;
  int pointId;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZSIFT           // here we assume a XYZ + "test" (as fields)
                                   ,(float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float[128], descriptor, descriptor)
                                   (int, multiplicity, multiplicity)
                                   (int, pointId, pointId)
)


//} //: namespace Types



#endif /* POINTXYZSIFT_HPP_ */
