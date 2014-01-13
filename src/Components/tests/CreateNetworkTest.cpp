#define BOOST_TEST_MODULE CreateNetworkTest
#include <boost/test/included/unit_test.hpp>

#include "CreateNetwork.hpp"

BOOST_AUTO_TEST_SUITE( test1 )

BOOST_AUTO_TEST_CASE( test )
{
    Processors::Network::CreateNetwork* network = new Processors::Network::CreateNetwork("TestNetwork");
    int temp = network->getTemp();
    BOOST_CHECK_EQUAL(4, temp);
    delete network;
}

BOOST_AUTO_TEST_SUITE_END()
