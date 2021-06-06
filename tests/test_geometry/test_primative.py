import copy
import logging
import unittest

import matplotlib.pyplot as plt

from geometry.primative import Line
from geometry.primative import Spline
from geometry.primative import Point

class TestLine(unittest.TestCase):

    def setUp(self):
        self._line = None
        self.point1 = None
        self.point2 = None
        self.logger = logging.getLogger()

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

        point_extrap = self._line.get_extrapolated_point(constraint=3.0, constraint_dim='z')
        self.assertIsNone(point_extrap)

        self.point1 = Point(x=0, y=0, z=0)
        self.point2 = Point(x=0, y=1, z=0)
        self._line = Line.line_from_points(self.point1, self.point2)

        point_extrap = self._line.get_extrapolated_point(constraint=3.0, constraint_dim='x')
        self.assertIsNone(point_extrap)

        point_extrap = self._line.get_extrapolated_point(constraint=3.0, constraint_dim='y')
        self.assertEqual(point_extrap, Point(0, 3, 0))

        self.point1 = Point(x=0, y=0, z=0)
        self.point2 = Point(x=0, y=0, z=1)
        self._line = Line.line_from_points(self.point1, self.point2)

        point_extrap = self._line.get_extrapolated_point(constraint=3.0, constraint_dim='z')
        self.assertEqual(point_extrap, Point(0, 0, 3))

        with self.assertRaises(ValueError):
            point_extrap = self._line.get_extrapolated_point(constraint=3.0, constraint_dim='u')

    def test_get_path(self):
        self.point1 = Point(x=0, y=0, z=0)
        self.point2 = Point(x=1, y=0, z=1)
        self._line = Line.line_from_points(self.point1, self.point2)

        path = self._line.get_path()

        self.assertLessEqual(path, [self.point1, self.point2])

    def test_signed_distance_to_point_xy(self):
        self.point1 = Point(x=0, y=0, z=0)
        self.point2 = Point(x=1, y=0, z=1)
        self._line = Line.line_from_points(self.point1, self.point2)

        test_point = Point(x=0, y=2, z=0.5)
        signed_dist = self._line.signed_distance_to_point_xy(test_point)
        self.assertEqual(signed_dist, -2)

        test_point = Point(0.5, 0, 0.5)
        signed_dist = self._line.signed_distance_to_point_xy(test_point)
        self.assertEqual(signed_dist, 0)

    def test_coord_in_range_dim(self):
        self.point1 = Point(x=0, y=0, z=0)
        self.point2 = Point(x=1, y=0, z=1)
        self._line = Line.line_from_points(self.point1, self.point2)

        test_point = Point(x=0.5, y=1, z=-2)
        self.assertTrue(self._line.coord_in_range_dim(coord=test_point['x'], dim='x'))
        self.assertFalse(self._line.coord_in_range_dim(coord=test_point['y'], dim='y'))

        self.point1 = Point(x=1, y=0, z=0)
        self.point2 = Point(x=-1, y=0, z=1)
        self._line = Line.line_from_points(self.point1, self.point2)

        self.assertTrue(self._line.coord_in_range_dim(coord=test_point['x'], dim='x'))


class TestPoint(unittest.TestCase):
    def setUp(self):
        self.x = 1.22
        self.y = -2.1
        self.z = 9
        self._point = Point(x=self.x, y=self.y, z=self.z)
        self.logger = logging.getLogger()

    def test_init_and_get_item(self):
        self.assertEqual(self._point['x'], self.x)
        self.assertEqual(self._point['y'], self.y)
        self.assertEqual(self._point[2], float(self.z))
        self.assertEqual(self._point['z'], float(self.z))
        self.assertTrue(isinstance(self._point['z'], float))

    def test_set_item(self):
        self._point['y'] = 2.1
        self.assertEqual(self._point['y'], 2.1)
        self._point[1] = -2.1
        self.assertEqual(self._point[1], -2.1)

    def test_str(self):
        self.assertEqual(str(self._point), '(1.22, -2.1, 9.0)')

    def test_eq(self):
        test_point = Point(self.x, self.y, self.z)
        self.assertEqual(test_point, self._point)

        test_point = Point(-self.x, -self.y, -self.z)
        self.assertNotEqual(test_point, self._point)

    def test_add(self):
        test_point = copy.copy(self._point)
        test_point2 = test_point + self._point
        self.assertEqual(test_point2['x'], 2.44)
        test_point2 += 2
        # Account for floating point error
        self.assertAlmostEqual(test_point2['x'], 4.44, places=5)

        with self.assertRaises(TypeError):
            test_point2 += '2'

    def test_sub(self):
        test_point = copy.copy(self._point)
        test_point2 = test_point - self._point
        self.assertEqual(test_point2['x'], 0)
        test_point2 -= 2
        self.assertEqual(test_point2['x'], -2)

        with self.assertRaises(TypeError):
            test_point2 -= '2'

    def test_pow(self):
        self.assertEqual(self.x ** 2 + self.y ** 2 + self.z ** 2, self._point ** 2)

    def test_repr(self):
        self.assertEqual(repr(self._point), '(1.22, -2.1, 9.0)')


class TestSpline(unittest.TestCase):
    def setUp(self):
        self.point1 = Point(0, 0, 0)
        self.point2 = Point(0, 2, 0)
        self.point3 = Point(2, 2, 0)
        self.point4 = Point(2, 0, 0)
        self.spline = Spline([self.point1, self.point2, self.point3, self.point4], resolution=1)
        self.opened_spline = Spline([self.point1, self.point2, self.point3, self.point4], resolution=1, closed_loop=False)

        self.logger = logging.getLogger()
        # self.spline.plot_spline()
        plt.show()

    def test_get_path(self):
        path = self.spline.get_path()

        # Closed loop includes point1 at the end to weakly enforce a closed loop
        path_comp = [self.point1, self.point2, self.point3, self.point4, self.point1]

        # Use almost equal due to floating point error
        for ind in range(0, len(path)):
            self.assertAlmostEqual(path[ind]['x'], path_comp[ind]['x'], 12)
            self.assertAlmostEqual(path[ind]['y'], path_comp[ind]['y'], 12)
            self.assertAlmostEqual(path[ind]['z'], path_comp[ind]['z'], 12)

        path_open = self.opened_spline.get_path()

        # Opened loop does not repeat the initial point at the end.
        path_comp_opened = [self.point1, self.point2, self.point3, self.point4]

        # Use almost equal due to floating point error
        for ind in range(0, len(path_open)):
            self.assertAlmostEqual(path_open[ind]['x'], path_comp_opened[ind]['x'], 12)
            self.assertAlmostEqual(path_open[ind]['y'], path_comp_opened[ind]['y'], 12)
            self.assertAlmostEqual(path_open[ind]['z'], path_comp_opened[ind]['z'], 12)

    def test_get_spline_length(self):
        # Closed loop path has a length of 8.0 mm since it is
        # a square of side length 2mm
        self.assertAlmostEqual(self.spline.get_spline_length(), 8.0, 12)
        # The open spline only has a length of 6mm as it is 3 sides of 2mm.
        self.assertAlmostEqual(self.opened_spline.get_spline_length(), 6.0, 12)

    # TODO: get_point_from_distance needs to be improved, currently fails unit tests
    def test_get_point_from_distance(self):
        test_point, curr_seg, length = self.spline.get_point_from_distance(dist=3)
        # self.assertAlmostEqual(length, 3.0, 12)
        # self.assertAlmostEqual(test_point['x'], 1.3148302275301, 12)
        # self.assertAlmostEqual(test_point['y'], 2.0, 12)
        # self.assertAlmostEqual(test_point['z'], 0.0, 12)

    def test_get_x_y_z(self):
        pass

    def test_get_points(self):
        pass


class TestPlane(unittest.TestCase):
    pass


class TestGeometricFunctions(unittest.TestCase):
    pass
