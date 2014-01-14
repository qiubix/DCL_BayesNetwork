#define BOOST_TEST_MODULE CreateNetworkTest
#include <boost/test/included/unit_test.hpp>

#include "CreateNetwork.hpp"

struct CreateNetworkTest
{
    Processors::Network::CreateNetwork* network;
    std::vector<std::string> outcomesNames;
    std::vector<std::string> parentsNames;
    vector<double> probabilities;
    CreateNetworkTest() {
        network = new Processors::Network::CreateNetwork("TestNetwork");
        outcomesNames.push_back("YES");
        outcomesNames.push_back("NO");
        probabilities.push_back(0.4);
        probabilities.push_back(0.6);
    }
    ~CreateNetworkTest(){
        delete network;
    }
};

BOOST_AUTO_TEST_SUITE( CreateNetworkTestSuite )

BOOST_FIXTURE_TEST_CASE( testAddNode, CreateNetworkTest )
{
    network->addNode("F1", outcomesNames, parentsNames);
    int node = network->getNetwork().FindNode("F1");
    const char* nodeTypeName = network->getNetwork().GetNode(node)->Definition()->GetTypeName();
    int numberOfOutcomes = network->getNetwork().GetNode(node)->Definition()->GetNumberOfOutcomes();
    BOOST_CHECK_EQUAL("CPT", nodeTypeName);
    BOOST_CHECK_EQUAL(0, node);
    BOOST_CHECK_EQUAL(2, numberOfOutcomes);
}

BOOST_FIXTURE_TEST_CASE( testSetNodeCPT, CreateNetworkTest )
{
    network->addNode("F1", outcomesNames, parentsNames);
    int node = network->getNetwork().FindNode("F1");
    BOOST_CHECK_EQUAL(0, node);
//    network->setNodeCPT("F1",probabilities);
//    DSL_sysCoordinates coords(*network->getNetwork().GetNode(node)->Value());
//    BOOST_CHECK_EQUAL(2,2);
//    DSL_idArray* names = network->getNetwork().GetNode(node)->Definition()->GetOutcomesNames();
//    BOOST_CHECK_EQUAL(2,2);
//    coords[0] = names->FindPosition("YES");
//    BOOST_CHECK_EQUAL(2,2);
//    coords.GoToCurrentPosition();
//    BOOST_CHECK_EQUAL(2,2);
//    double probability = coords.UncheckedValue();
//    BOOST_CHECK_EQUAL(0.4, probability);
}

BOOST_AUTO_TEST_SUITE_END()

/*
 * TODO: write test cases:
 * testSetNodeCPT
 * testCreateJointMultiplitityVector
 * testSetBaseFeaturesCPTs
 * testsetBaseHypothesesCPTs
 */
