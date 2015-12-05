#include <gmock/gmock.h>
using ::testing::Eq;
#include <gtest/gtest.h>
using ::testing::Test;

#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

//#include "../src/Components/NetworkBuilder/Octree.hpp"

//TODO: FIXME: include types from PCL
//#include <Types/PointXYZSIFT>
#include "../src/Types/PointXYZSIFT.hpp"

TEST(OctreeBuilderTest, shouldTestNothing) {
  ASSERT_THAT(1, Eq(1));
}
