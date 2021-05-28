import os
import unittest

import coverage

def run_unitests():
    suite = unittest.TestLoader().discover(start_dir=os.path.dirname(__file__),pattern='test_*.py')
    print(suite)
    unittest.TextTestRunner(verbosity=2).run(suite)

def run_unittest_with_coverage():
    cov = coverage.Coverage()
    cov.start()

    run_unitests()

    cov.stop()
    cov.save()

    cov.html_report(directory=os.path.join(os.path.dirname(os.path.dirname(__file__)), 'coverage'))


run_unittest_with_coverage()
