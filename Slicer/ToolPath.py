import numpy as np
import matplotlib.pyplot as plt

from Geometry.PrimativeGeometry import Spline
from Geometry.PrimativeGeometry import Point
from Geometry.PrimativeGeometry import Line
from Slicer.WireCutter import WireCutter


class ToolPath(object):
    """Class defining tool path motion for dual gantry cnc wire cutters. Consists of lists of points for Gantry 1 and
    Gantry 2.

    :param self._path1: Gantry 1 path.
    :type self._path1: list[Point]
    :param self._path2: Gantry 2 path.
    :type self._path2: list[Point]

    """

    def __init__(self, path1, path2):
        """Constructor for tool path, should not be directly called, use create_* functions to generate tool paths.

        :param path1: Gantry 1 path.
        :type path1: list[Point]
        :param path2: Gantry 2 path.
        :type path2: list[Point]
        """
        self._path1 = path1
        self._path2 = path2

    def plot_tool_paths(self):
        """PLots the two gantry tool paths as 2d paths on the x-y plane."""
        x1 = np.zeros(len(self._path1))
        x2 = np.zeros(len(self._path1))
        y1 = np.zeros(len(self._path1))
        y2 = np.zeros(len(self._path1))

        for i in range(0, len(self._path1)):
            x1[i] = self._path1[i]['x']
            x2[i] = self._path2[i]['x']
            y1[i] = self._path1[i]['y']
            y2[i] = self._path2[i]['y']

        plt.plot(x1, y1)
        plt.plot(x2, y2)

    @staticmethod
    def create_tool_path_from_two_splines(spline1, spline2, wire_cutter):
        """Creates a tool path from two splines and a wire_cutter object.

        :param Spline spline1: Spline for cut profile 1.
        :param Spline spline2: Spline for cut profile 2.
        :param WireCutter wire_cutter: Definition of the Wire cutter machine.
        :return: Tool path to cut the two cut profiles.
        """
        # Get the x,y,z coordinate of each spline. Spline resolution is defaulted to 10x more points than the
        # underlying point cloud. Spline resolution can be modified to evenly load the two path lengths
        (x1, y1, z1) = spline1.get_x_y_z()
        (x2, y2, z2) = spline2.get_x_y_z()

        len_x1 = len(x1)
        len_x2 = len(x2)

        path1 = list()
        path2 = list()

        # interpolate the spline with less data points to be the same length as the other
        if len_x1 > len_x2:
            print(len_x1, len_x2)
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
            point1 = Point(x1[val], y1[val], z1[val])
            point2 = Point(x2[val], y2[val], z2[val])

            line = Line.line_from_points(point1, point2)

            gantry1_point = line.get_extrapolated_point(0, 'z')
            gantry2_point = line.get_extrapolated_point(wire_cutter.wire_length, 'z')

            path1.append(gantry1_point)
            path2.append(gantry2_point)

        return ToolPath(path1, path2)
