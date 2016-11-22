#pragma once
#ifndef DEFAULT_SIFT_POINT_REPRESENTATION_HPP
#define DEFAULT_SIFT_POINT_REPRESENTATION_HPP

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
//#include "pcl/impl/instantiate.hpp"

//#include <boost/preprocessor.hpp>

namespace pcl{
template<>
class DefaultPointRepresentation<PointXYZSIFT> : public PointRepresentation<PointXYZSIFT>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 3;
    trivial_ = true;
  }

  virtual void
  copyToFloatArray (const PointXYZSIFT &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
  }
};
}

#endif  //DEFAULT_SIFT_POINT_REPRESENTATION_HPP
