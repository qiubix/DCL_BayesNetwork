#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include "Components/OctreeBuilder/OctreeBuilder.hpp"

#include "Types/PointXYZSIFT.hpp"
#include "Types/Octree.hpp"

using Processors::Network::OctreeBuilder;

class OctreeBuilderTest : public Test {
public:
  OctreeBuilderTest() {
    builder = new OctreeBuilder("builder");
  }

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

  PointCloud getPointCloudWithThreePoints() {
    PointCloud cloud(new pcl::PointCloud<PointXYZSIFT>);
    cloud -> width = 3;
    cloud -> height = 1;
    cloud -> points.resize(cloud -> width * cloud -> height);
    cloud -> points[0].x = 0.0;
    cloud -> points[0].y = 0.0;
    cloud -> points[0].z = 0.0;
    cloud -> points[0].pointId = 0;
    cloud -> points[1].x = 1.0;
    cloud -> points[1].y = 1.0;
    cloud -> points[1].z = 1.0;
    cloud -> points[1].pointId = 2;
    cloud -> points[2].x = 0.0;
    cloud -> points[2].y = 0.0;
    cloud -> points[2].z = 1.0;
    cloud -> points[2].pointId = 4;
    for(int i=0; i<128; i++) {
      cloud -> points[0].descriptor[i] = i;
      cloud -> points[1].descriptor[i] = 128-i;
      cloud -> points[2].descriptor[i] = i*i;
    }
    return cloud;
  }

  OctreeBuilder* builder;
};

TEST_F(OctreeBuilderTest, shouldAcceptPointCloudWithSIFT) {
  builder -> setPointCloud(getPointCloudWithOnePoint());

  unsigned long numberOfPoints = builder -> getPointCloud()->points.size();
  ASSERT_THAT(numberOfPoints, Eq(1));
}

TEST_F(OctreeBuilderTest, shouldReturnOctreeWithSameNumberOfPointsAsPointCloud) {
  PointCloud pointCloud = getPointCloudWithThreePoints();
  builder -> setPointCloud(pointCloud);

  builder -> buildOctree();

  Processors::Network::Octree octree = builder -> getOctree();
  ASSERT_THAT(octree.getNumberOfPoints(), Eq(pointCloud -> size()));
}

TEST_F(OctreeBuilderTest, shouldHaveLeafsWithAtMostEightPoints) {
  //TODO
}

TEST_F(OctreeBuilderTest, shouldReturnOctreeWithProperResolution) {
  //TODO
}

TEST_F(OctreeBuilderTest, shouldInitializeComponentHandlers) {
  builder -> prepareInterface();
  
  std::string handlers = builder -> listHandlers();
  ASSERT_THAT(handlers, Eq("onNewCloud\n"));
}
