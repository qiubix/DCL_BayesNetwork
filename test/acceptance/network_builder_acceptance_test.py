import unittest
from hamcrest import *
from discoderunner import *


class NetworkBuilderAcceptanceTest(unittest.TestCase):

    def test_should_run_dummy_test(self):
        assert_that(1, equal_to(1))

    def test_should_load_network_builder_component(self):
        tester = ComponentTester()
        tester.addGenerator('BayesNetworkGenerators:OctreeGenerator')
        tester.setComponent('NetworkBuilder', 'BayesNetwork:NetworkBuilder')
        tester.addSink('BayesNetworkGenerators:BayesNetworkSink')
        tester.addDataStream('Generator', 'out_octree', 'NetworkBuilder', 'in_octree')
        tester.addDataStream('NetworkBuilder', 'out_network', 'Sink', 'in_network')
        tester.setTerminationStatement('END OF SEQUENCE')

        tester.start()
        output = tester.getOutput()
        print(output)
        assert_that(output, contains_string('Number of feature nodes:'))

if __name__ == '__main__':
    unittest.main(warnings='ignore', verbosity=2)
