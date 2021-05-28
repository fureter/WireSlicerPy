import unittest

from geometry.primative import Line
from geometry.primative import Point

class TestLine(unittest.TestCase):

    def setUp(self):
        self._line = None
        self.point1 = None
        self.point2 = None

    def test_line_from_two_points(self):
        self.point1 = Point(x=0, y=0, z=0)
        self.point2 = Point(x=1, y=0, z=-1)

        self._line = Line.line_from_points(self.point1, self.point2)
        self.assertEqual(self._line._x0, self.point1['x'])
        self.assertEqual(self._line._y0, self.point1['y'])
        self.assertEqual(self._line._z0, self.point1['z'])

    def test_get_extrapolated_point(self):
        self.point1 = Point(x=0, y=0, z=0)
        self.point2 = Point(x=2, y=0, z=0)
        self._line = Line.line_from_points(self.point1, self.point2)

        point_extrap = self._line.get_extrapolated_point(constraint=3.0, constraint_dim='x')
        self.assertEqual(point_extrap, Point(3, 0, 0))

        point_extrap = self._line.get_extrapolated_point(constraint=3.0, constraint_dim='y')
        self.assertIsNone(point_extrap)


class TestPoint(unittest.TestCase):
    pass

class TestSpline(unittest.TestCase):
    pass

class TestPlane(unittest.TestCase):
    pass

class TestGeometricFunctions(unittest.TestCase):
    pass

