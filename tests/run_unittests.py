import argparse
import logging
import os
import unittest

import coverage

import wire_slicer


def run_unitests():
    suite = unittest.TestLoader().discover(start_dir=os.path.dirname(__file__), pattern='test_*.py')
    print(suite)
    unittest.TextTestRunner(verbosity=2, buffer=False).run(suite)


def run_unittest_with_coverage():
    cov = coverage.Coverage(source=[os.path.dirname(os.path.dirname(__file__))], omit=['test_*.py', '*unittests.py'])
    cov.start()

    run_unitests()

    cov.stop()
    cov.save()

    cov.html_report(directory=os.path.join(os.path.dirname(os.path.dirname(__file__)), 'coverage'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-cov', action='store_true', help='Enable coverage report')
    cov = vars(parser.parse_args())['cov']

    # Setup the project and setup the logger to be used throughout unit tests.
    wire_slicer.setup(logging.DEBUG)
    if cov:
        run_unittest_with_coverage()
    else:
        run_unitests()