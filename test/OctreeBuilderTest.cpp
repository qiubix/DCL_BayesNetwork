#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "../src/Components/OctreeBuilder/OctreeBuilder.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"
#include "../src/Types/Octree.hpp"

using Processors::Network::OctreeBuilder;

class OctreeBuilderTest : public Test {
public:
  typedef pcl::PointCloud<PointXYZSIFT>::Ptr PointCloud;

  PointCloud getPointCloudWithOnePoint() {
    PointCloud cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud->width = 1;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->points[0].x = 0.1;
    cloud->points[0].y = 0.2;
    cloud->points[0].z = 0.3;
    for(int i=0; i<128; i++) cloud->points[0].descriptor[i] = i;
    return cloud;
  }
};

TEST_F(OctreeBuilderTest, shouldAcceptPointCloudWithSIFT) {
  OctreeBuilder builder("builder");

  builder.setPointCloud(getPointCloudWithOnePoint());
  unsigned long numberOfPoints = builder.getPointCloud()->points.size();
  ASSERT_THAT(numberOfPoints, Eq(1));
}

TEST_F(OctreeBuilderTest, shouldReturnOctreeWithOnePoint) {
  OctreeBuilder builder("builder");
  builder.setPointCloud(getPointCloudWithOnePoint());

  builder.buildOctree();

  Processors::Network::Octree octree = builder.getOctree();
  ASSERT_THAT(octree.getNumberOfPoints(), Eq(1));
}
