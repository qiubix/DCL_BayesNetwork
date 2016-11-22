import unittest
from hamcrest import *
from discoderunner import *


class SOMEvaluationAcceptanceTest(unittest.TestCase):

    def test_should_run_tests(self):
        assert_that(1, equal_to(1))

    def test_should_load_som_evaluation_component(self):
        tester = ComponentTester()

if __name__ == '__main__':
    unittest.main(warnings='ignore', verbosity=2)
