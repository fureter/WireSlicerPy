import copy
import logging
import unittest

import numpy as np
import matplotlib.pyplot as plt

from geometry.primative import Line
from geometry.primative import Spline
from geometry.primative import Point
from geometry.primative import GeometricFunctions

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
        self.points = list()
        self.radius = 1000.0
        self.num_points = 100
        self.resolution = 5
        self.origin = np.array([0, 0, 0])
        for ind in range(self.num_points):
            self.points.append(Point(self.origin[0] + np.cos(2*np.pi*ind/100.0)*self.radius,
                                     self.origin[1] + np.sin(2*np.pi*ind/100.0)*self.radius,
                                     self.origin[2]))
        self.spline = Spline(self.points, resolution=self.resolution)
        self.opened_spline = Spline(self.points, resolution=self.resolution, closed_loop=False)

        self.logger = logging.getLogger(__name__)

    def tearDown(self):
        pass

    def test_get_path(self):

        path_open = self.opened_spline.get_path()

        # Closed loop includes point1 at the end to weakly enforce a closed loop
        path_comp = copy.deepcopy(self.points)

        # Use almost equal due to floating point error
        for ind in range(0, len(path_open)*self.resolution):
            self.assertAlmostEqual(path_open[ind]['x'], path_comp[int(ind/self.resolution)]['x'], 12)
            self.assertAlmostEqual(path_open[ind]['y'], path_comp[int(ind/self.resolution)]['y'], 12)
            self.assertAlmostEqual(path_open[ind]['z'], path_comp[int(ind/self.resolution)]['z'], 12)

        path = self.spline.get_path()

        # Closed loop includes point1 at the end to weakly enforce a closed loop
        path_comp.append(path_comp[0])

        # Use almost equal due to floating point error
        for ind in range(0, len(path)):
            self.assertAlmostEqual(path[ind]['x'], path_comp[int(ind/self.resolution)]['x'], 12)
            self.assertAlmostEqual(path[ind]['y'], path_comp[int(ind/self.resolution)]['y'], 12)
            self.assertAlmostEqual(path[ind]['z'], path_comp[int(ind/self.resolution)]['z'], 12)

    def test_get_spline_length(self):
        # Closed loop path has a length of 8.0 mm since it is
        # a square of side length 2mm
        self.logger.debug("Expected circumference: %s", 2*np.pi*self.radius)
        self.assertAlmostEqual(self.spline.get_spline_length(), 2*np.pi*self.radius, 12)
        # The open spline only has a length of 6mm as it is 3 sides of 2mm.
        self.assertAlmostEqual(self.opened_spline.get_spline_length(), 2*np.pi*self.radius*(1-1/self.num_points), 12)

    # TODO: get_point_from_distance needs to be improved, currently fails unit tests
    def test_get_point_from_distance(self):
        test_point, curr_seg, length = self.spline.get_point_from_distance(dist=3)
        self.assertAlmostEqual(length, 3.0, 12)
        self.assertAlmostEqual(test_point['x'], 1.3148302275301, 12)
        self.assertAlmostEqual(test_point['y'], 2.0, 12)
        self.assertAlmostEqual(test_point['z'], 0.0, 12)

    def test_get_x_y_z(self):
        x, y, z = self.spline.get_x_y_z(resolution=2)

        radius = np.sqrt(x**2 + y**2 + z**2)
        self.assertTrue(np.all(np.abs(radius - self.radius) < 1e-3))

    def test_get_points(self):
        points = self.spline.get_points(resolution=2)
        for point in points:
            radius = np.sqrt(point**2)
            self.logger.debug(np.abs(radius - self.radius) < 1e-3)
            self.logger.debug(np.abs(radius - self.radius))
            self.assertTrue(np.all(np.abs(radius - self.radius) < 1e-3))


class TestPlane(unittest.TestCase):
    pass


class TestGeometricFunctions(unittest.TestCase):
    pass
