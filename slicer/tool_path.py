import logging
import timeit

import numpy as np
import matplotlib.pyplot as plt

# from geometry.primative import Spline
# from geometry.primative import Point
# from geometry.primative import Line
# from geometry.primative import GeometricFunctions
import geometry.primative as prim
import geometry.complex as compx
from geometry.spatial_manipulation import PointManip
from slicer.wire_cutter import WireCutter


class ToolPath():
    """Class defining tool path motion for dual gantry cnc wire cutters. Consists of lists of points for Gantry 1 and
    Gantry 2.

    :param self._path1: Gantry 1 path.
    :type self._path1: list[Point]
    :param self._path2: Gantry 2 path.
    :type self._path2: list[Point]

    """

    def __init__(self, path1, path2, logger=None):
        """Constructor for tool path, should not be directly called, use create_* functions to generate tool paths.

        :param path1: Gantry 1 path.
        :type path1: list[prim.Point]
        :param path2: Gantry 2 path.
        :type path2: list[prim.Point]
        """
        self._path1 = path1
        self._path2 = path2

        if logger is None:
            logger = logging.getLogger()
        self.logger = logger

    def zero_forwards_path_for_cutting(self):
        """
        Shifts the points of both gantry paths so that the most forward path is at X=0.

        :return:
        """

        minimum_point = min([prim.GeometricFunctions.get_point_from_min_coord(self._path1, 'x')['x'],
                             prim.GeometricFunctions.get_point_from_min_coord(self._path2, 'x')['x']])

        offset = -minimum_point

        PointManip.Transform.translate(self._path1, [offset, 0, 0])
        PointManip.Transform.translate(self._path2, [offset, 0, 0])

    def plot_tool_paths(self):
        """PLots the two gantry tool paths as 2d paths on the x-y plane."""
        len_path1 = len(self._path1)
        len_path2 = len(self._path2)

        x1 = np.zeros(len_path1)
        x2 = np.zeros(len_path2)
        y1 = np.zeros(len_path1)
        y2 = np.zeros(len_path2)

        for i in range(0, len_path1):
            x1[i] = self._path1[i]['x']
            y1[i] = self._path1[i]['y']

        for i in range(0, len_path2):
            x2[i] = self._path2[i]['x']
            y2[i] = self._path2[i]['y']

        plt.plot(x1, y1)
        plt.plot(x2, y2)
        plt.scatter(x1, y1)
        plt.scatter(x2, y2)

    def plot_tool_path_connections(self, step):
        """PLots the two gantry tool paths as 2d paths on the x-y plane."""
        len_path1 = len(self._path1)
        len_path2 = len(self._path2)

        if abs(len_path1 - len_path2) > 2:
            raise AttributeError('Error: Both tool paths are not of equal length [Path1: %s, Path2: %s]' % (len_path1,
                                                                                                            len_path2))

        for i in range(0, len_path1, step):
            if i < len_path2:
                x = (self._path1[i]['x'], self._path2[i]['x'])
                y = (self._path1[i]['y'], self._path2[i]['y'])
                plt.plot(x, y)

    @staticmethod
    def create_tool_path_from_two_gantry_paths(path1, path2):
        return ToolPath(path1, path2)

    @staticmethod
    def create_tool_path_from_wing_segment(wing, wire_cutter):
        """

        :param compx.WingSegment wing:
        :return:
        """
        return ToolPath.create_tool_path_from_paths(wing.root_airfoil, wing.tip_airfoil, wire_cutter=wire_cutter)

    @staticmethod
    def create_tool_path_from_two_splines(spline1, spline2, wire_cutter):
        """Creates a tool path from two splines and a wire_cutter .

        :param Spline spline1: Spline for cut profile 1.
        :param Spline spline2: Spline for cut profile 2.
        :param WireCutter wire_cutter: Definition of the Wire cutter machine.
        :return: Tool path to cut the two cut profiles.
        """
        # Get the x,y,z coordinate of each spline. Spline resolution is defaulted to 10x more points than the
        # underlying point cloud. Spline resolution can be modified to evenly load the two path lengths
        (x1, y1, z1) = spline1.get_x_y_z()
        (x2, y2, z2) = spline2.get_x_y_z()

        return ToolPath.create_tool_path_from_paths(zip(x1, y1, z1), zip(x2, y2, z2), wire_cutter=wire_cutter)

    @staticmethod
    def create_tool_path_from_paths(path1, path2, wire_cutter):
        x1, y1, z1 = zip(*path1)
        x2, y2, z2 = zip(*path2)

        len_x1 = len(x1)
        len_x2 = len(x2)

        path1 = list()
        path2 = list()

        # interpolate the spline with less data points to be the same length as the other
        if len_x1 > len_x2:
            t = list(range(0, len_x1))
            t_base = list(range(0, len_x2))
            scale = t[-1] / t_base[-1]
            for i in range(0, len_x2):
                t_base[i] *= scale

            x2 = np.interp(t, t_base, x2)
            y2 = np.interp(t, t_base, y2)
            z2 = np.interp(t, t_base, z2)
        else:
            t = list(range(0, len_x2))
            t_base = list(range(0, len_x1))
            scale = t[-1] / t_base[-1]
            for i in range(0, len_x1):
                t_base[i] *= scale

            x1 = np.interp(t, t_base, x1)
            y1 = np.interp(t, t_base, y1)
            z1 = np.interp(t, t_base, z1)

        # for every point pair in point list 1 and 2, form a line connecting the two, and interpolate this line to the
        # two gantry plane positions to get the points along the gantry tool path
        for val in t:
            point1 = prim.Point(x1[val], y1[val], z1[val])
            point2 = prim.Point(x2[val], y2[val], z2[val])

            line = prim.Line.line_from_points(point1, point2)
            gantry1_point = line.get_extrapolated_point(0, 'z')
            gantry2_point = line.get_extrapolated_point(wire_cutter.wire_length, 'z')

            path1.append(gantry1_point)
            path2.append(gantry2_point)

        return ToolPath(path1, path2)

    def _shortest_distance_in_tool_path(self):
        min_dist = 99999999
        for i in range(0, len(self._path1) - 1):
            dist = np.sqrt((self._path1[i + 1] - self._path1[i]) ** 2)
            if dist < min_dist:
                min_dist = dist

        for i in range(0, len(self._path2) - 1):
            dist = np.sqrt((self._path2[i + 1] - self._path2[i]) ** 2)
            if dist < min_dist:
                min_dist = dist

        return min_dist

    def get_key_points_for_wing(self):
        key_points = list()
        point1, _ = prim.GeometricFunctions.get_point_from_max_coord(self._path1, 'x')
        point2, _ = prim.GeometricFunctions.get_point_from_max_coord(self._path2, 'x')
        key_points.append((point1, point2))
        return key_points

    def _longest_distance_in_tool_path(self):
        max_dist = 0
        for i in range(0, len(self._path1) - 1):
            dist = np.sqrt((self._path1[i + 1] - self._path1[i]) ** 2)
            if dist > max_dist:
                max_dist = dist

        for i in range(0, len(self._path2) - 1):
            dist = np.sqrt((self._path2[i + 1] - self._path2[i]) ** 2)
            if dist > max_dist:
                max_dist = dist

        return max_dist

    def create_path_with_uniform_point_distances(self):
        start = timeit.default_timer()
        min_dist = self._shortest_distance_in_tool_path()
        min_dist = 1
        self.logger.info('Took %ss to find the minimum spacing in the toolpath' % (timeit.default_timer() - start))
        self.logger.info('Minimum distance in either path is: %smm' % min_dist)

        self.logger.info('Getting Lengths of each path')
        start = timeit.default_timer()
        length_path1 = prim.GeometricFunctions.path_length(self._path1)
        self.logger.info('Took %ss to calculate length of path 1' % (timeit.default_timer() - start))
        start = timeit.default_timer()
        length_path2 = prim.GeometricFunctions.path_length(self._path2)

        self.logger.info('Took %ss to calculate length of path 2' % (timeit.default_timer() - start))

        self.logger.info('Path 1 Length: %smm' % length_path1)
        self.logger.info('Path 2 Length: %smm' % length_path2)

        path1 = list()
        path2 = list()

        path1.append(self._path1[0])
        path2.append(self._path2[0])

        curr_dist = 0
        idx = 0
        length = 0
        start = timeit.default_timer()
        while curr_dist < length_path1:
            curr_dist += min_dist
            point = prim.GeometricFunctions.get_point_along_path(self._path1, curr_dist)
            path1.append(point)
        self.logger.info('Took %ss get movement points from path 1' % (timeit.default_timer() - start))

        curr_dist = 0
        idx = 0
        length = 0
        start = timeit.default_timer()
        while curr_dist < length_path2:
            curr_dist += min_dist
            point = prim.GeometricFunctions.get_point_along_path(self._path2, curr_dist)
            path2.append(point)
        self.logger.info('Took %ss get movement points from path 2' % (timeit.default_timer() - start))

        return (path1, path2)

    def create_path_with_uniform_point_distances_splines(self):
        start = timeit.default_timer()
        min_dist = self._shortest_distance_in_tool_path()
        self.logger.info('Took %ss to find the minimum spacing in the toolpath' % (timeit.default_timer() - start))
        min_dist = 1
        self.logger.info('Minimum distance in either path is: %smm' % min_dist)

        start = timeit.default_timer()
        spline1 = prim.Spline(self._path1)
        spline2 = prim.Spline(self._path2)
        self.logger.info('Took %ss to create splines for path 1 and path 2' % (timeit.default_timer() - start))

        self.logger.info('Getting Lengths of each path')

        start = timeit.default_timer()
        length_spline1 = spline1.get_spline_length()
        self.logger.info('Took %ss to calculate length of path 1' % (timeit.default_timer() - start))

        start = timeit.default_timer()
        length_spline2 = spline2.get_spline_length()
        self.logger.info('Took %ss to calculate length of path 2' % (timeit.default_timer() - start))
        self.logger.info('Path 1 Length: %smm' % length_spline1)
        self.logger.info('Path 2 Length: %smm' % length_spline2)

        path1 = list()
        path2 = list()

        curr_dist = 0
        point1, _, _ = spline1.get_point_from_distance(curr_dist)
        point2, _, _ = spline2.get_point_from_distance(curr_dist)
        path1.append(point1)
        path2.append(point2)

        self.logger.info('Creating new path by stepping along previous path in %smm '
                         'steps until reaching %smm total length' % (min_dist, length_spline1))
        t = 1E-1
        length = 0
        start = timeit.default_timer()
        while curr_dist < length_spline1:
            curr_dist += min_dist
            point, t, length = spline1.get_point_from_distance(curr_dist, t=t, dt=1E-1, start=length)
            path1.append(point)

        self.logger.info('Took %ss get movement points from path 1' % (timeit.default_timer() - start))

        t = 1E-1
        length = 0
        curr_dist = 0
        start = timeit.default_timer()
        while curr_dist < length_spline2:
            curr_dist += min_dist
            point, t, length = spline2.get_point_from_distance(curr_dist, t=t, dt=1E-1, start=length)
            path2.append(point)

        self.logger.info('Took %ss get movement points from path 2' % (timeit.default_timer() - start))

        return (path1, path2)

    def create_path_with_uniform_ratio_spacing(self, key_points):
        path1 = list()
        path2 = list()
        path1.append(self._path1[0])
        path2.append(self._path2[0])

        if key_points is None:
            path1_tmp, path2_tmp = self._get_uniform_spacing_points_along_path_segment(
                start_point=(self._path1[0], self._path2[0]),
                end_point=(self._path1[0], self._path2[0]))
            path1.extend(path1_tmp)
            path2.extend(path2_tmp)
        elif len(key_points) == 1:
            path1_tmp, path2_tmp = self._get_uniform_spacing_points_along_path_segment(
                start_point=(self._path1[0], self._path2[0]),
                end_point=key_points[0])

            length1 = len(path1_tmp)
            length2 = len(path2_tmp)
            min_len = min([length1, length2])

            path1.extend(path1_tmp[:min_len])
            path2.extend(path2_tmp[:min_len])

            path1_tmp, path2_tmp = self._get_uniform_spacing_points_along_path_segment(
                start_point=key_points[0],
                end_point=(self._path1[0], self._path2[0]))
            length1 = len(path1_tmp)
            length2 = len(path2_tmp)
            min_len = min([length1, length2])

            path1.extend(path1_tmp[:min_len])
            path2.extend(path2_tmp[:min_len])
        else:
            path1_tmp, path2_tmp = self._get_uniform_spacing_points_along_path_segment(
                start_point=(self._path1[0], self._path2[0]),
                end_point=key_points[0])
            path1.extend(path1_tmp)
            path2.extend(path2_tmp)

            for idx in range(0, len(key_points) - 1):
                path1_tmp, path2_tmp = self._get_uniform_spacing_points_along_path_segment(
                    start_point=key_points[idx],
                    end_point=key_points[idx + 1])
                path1.extend(path1_tmp)
                path2.extend(path2_tmp)

            path1_tmp, path2_tmp = self._get_uniform_spacing_points_along_path_segment(
                start_point=key_points[-1],
                end_point=(self._path1[0], self._path2[0]))
            path1.extend(path1_tmp)
            path2.extend(path2_tmp)

        path1.append(self._path1[0])
        path2.append(self._path2[0])

        return path1, path2

    def _get_uniform_spacing_points_along_path_segment(self, start_point, end_point):
        path1 = list()
        path2 = list()
        self.logger.info('Getting Lengths of each path')
        start = timeit.default_timer()
        length_path1 = prim.GeometricFunctions.path_length_from_point_to_point(self._path1, start_point[0], end_point[0])
        self.logger.info('Took %ss to calculate length of path 1' % (timeit.default_timer() - start))
        start = timeit.default_timer()
        length_path2 = prim.GeometricFunctions.path_length_from_point_to_point(self._path2, start_point[1], end_point[1])

        self.logger.info('Took %ss to calculate length of path 2' % (timeit.default_timer() - start))

        self.logger.info('Path 1 Length: %smm' % length_path1)
        self.logger.info('Path 2 Length: %smm' % length_path2)

        curr_dist = 0
        idx = 0
        length = 0
        start = timeit.default_timer()
        while curr_dist < length_path1:
            curr_dist += length_path1 / 100
            point = prim.GeometricFunctions.get_point_along_path(self._path1, curr_dist, start_point=start_point[0])
            path1.append(point)
        self.logger.info('Took %ss get movement points from path 1' % (timeit.default_timer() - start))

        curr_dist = 0
        idx = 0
        length = 0
        start = timeit.default_timer()
        while curr_dist < length_path2:
            curr_dist += length_path2 / 100
            point = prim.GeometricFunctions.get_point_along_path(self._path2, curr_dist, start_point=start_point[1])
            path2.append(point)
        self.logger.info('Took %ss get movement points from path 2' % (timeit.default_timer() - start))

        self.logger.info('Length of XY Move List: %s' % len(path1))
        self.logger.info('Length of UZ Move List: %s' % len(path2))
        return (path1, path2)
