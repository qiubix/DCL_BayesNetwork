import unittest
from hamcrest import *
from discoderunner import *


class NetworkBuilderAcceptanceTest(unittest.TestCase):

    def setUp(self):
        self.tester = ComponentTester()
        self.tester.addGenerator('BayesNetwork:OctreeGenerator')
        self.tester.setComponent('NetworkBuilder', 'BayesNetwork:NetworkBuilder')
        self.tester.addSink('BayesNetwork:BayesNetworkSink')
        self.tester.addDataStream('Generator', 'out_octree', 'NetworkBuilder', 'in_octree')
        self.tester.addDataStream('NetworkBuilder', 'out_network', 'Sink', 'in_network')
        self.tester.setTerminationStatement('END OF SEQUENCE')

    def test_should_build_network_with_8_feature_nodes(self):
        self.tester.setDebugMode(True)
        self.tester.start()

        output = self.tester.getOutput()
        print(output)
        assert_that(output, contains_string('Number of feature nodes: 8'))

    def test_should_build_network_with_8_leaf_nodes(self):
        self.tester.setDebugMode(True)
        self.tester.start()

        output = self.tester.getOutput()
        print(output)
        assert_that(output, contains_string('Leaf node quantity: 8'))

    def test_should_network_have_total_23_nodes(self):
        self.tester.setDebugMode(True)
        self.tester.start()

        output = self.tester.getOutput()
        print(output)
        assert_that(output, contains_string('Number of all nodes: 23'))


if __name__ == '__main__':
    unittest.main(warnings='ignore', verbosity=2)
