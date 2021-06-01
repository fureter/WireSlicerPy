import os
import unittest

import coverage

import wire_slicer


def run_unitests():
    suite = unittest.TestLoader().discover(start_dir=os.path.dirname(__file__),pattern='test_*.py')
    print(suite)
    unittest.TextTestRunner(verbosity=2).run(suite)


def run_unittest_with_coverage():
    # Setup the project and setup the logger to be used throughout unit tests.
    wire_slicer.setep()

    cov = coverage.Coverage()
    cov.start()

    run_unitests()

    cov.stop()
    cov.save()

    cov.html_report(directory=os.path.join(os.path.dirname(os.path.dirname(__file__)), 'coverage'))


run_unittest_with_coverage()
