#include <gmock/gmock.h>
using ::testing::Eq;
using ::testing::_;
using ::testing::Test;
using ::testing::Return;
using ::testing::NiceMock;

#include "Types/Octree.hpp"
#include "Types/OctreeDepthFirstIterator.hpp"
#include "utils/PointCloudGenerator.hpp"

TEST(OctreeIteratorTest, shouldGetFirstOctreeNode) {
  PointCloud cloud = getPointCloudWithThreePoints();
  Processors::Network::Octree octree(cloud);
  octree.init();

  Processors::Network::OctreeDepthFirstIterator it = octree.depthBegin();

  ASSERT_THAT(it->isBranchNode(), Eq(true));
  ASSERT_THAT(it->getCurrentOctreeDepth(), Eq(0));
}
