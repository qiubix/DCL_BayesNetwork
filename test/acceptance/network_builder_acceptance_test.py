import unittest
from hamcrest import *
from discoderunner import *


class NetworkBuilderAcceptanceTest(unittest.TestCase):

    def test_should_load_network_builder_component(self):
        tester = ComponentTester()
        tester.addGenerator('BayesNetwork:OctreeGenerator')
        tester.setComponent('NetworkBuilder', 'BayesNetwork:NetworkBuilder')
        tester.addSink('BayesNetwork:BayesNetworkSink')
        tester.addDataStream('Generator', 'out_octree', 'NetworkBuilder', 'in_octree')
        tester.addDataStream('NetworkBuilder', 'out_network', 'Sink', 'in_network')
        tester.setTerminationStatement('END OF SEQUENCE')

        tester.start()
        output = tester.getOutput()
        print(output)
        assert_that(output, contains_string('Number of feature nodes: 8'))

if __name__ == '__main__':
    unittest.main(warnings='ignore', verbosity=2)
