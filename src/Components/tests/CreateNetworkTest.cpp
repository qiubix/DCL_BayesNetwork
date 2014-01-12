#define BOOST_TEST_MODULE CreateNetworkTest
#include <boost/test/included/unit_test.hpp>

#include "CreateNetwork.hpp"

/*
 * TODO: write test case fixture
 */

BOOST_AUTO_TEST_CASE( test )
{
    Processors::Network::CreateNetwork* network = new Processors::Network::CreateNetwork("TestNetwork");
    BOOST_CHECK_EQUAL(2, 2);
}

/*
 * TODO: write test cases:
 * testAddNode
 * testSetNodeCPT
 * testCreateJointMultiplitityVector
 * testSetBaseFeaturesCPTs
 * testsetBaseHypothesesCPTs
 */
