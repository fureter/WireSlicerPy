import copy
import logging
import os.path
import timeit

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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

    def __init__(self, path1, path2, speed_list=None, logger=None):
        """Constructor for tool path, should not be directly called, use create_* functions to generate tool paths.

        :param path1: Gantry 1 path.
        :type path1: list[prim.Point]
        :param path2: Gantry 2 path.
        :type path2: list[prim.Point]
        """
        self._path1 = path1
        self._path2 = path2

        self.speed_list = speed_list

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

    @staticmethod
    def create_tool_path_from_cut_path(cut_path, wire_cutter, output_dir=None):
        """

        :param compx.CutPath cut_path:
        :param WireCutter wire_cutter:
        :return:
        """
        logger = logging.getLogger(__name__)
        path1w = list()
        path2w = list()
        path1 = list()
        path2 = list()
        speed_list = list()

        if not cut_path.is_valid_cut_path():
            raise ValueError('Error: Provided CutPath is not a valid CutPath')

        for idx in range(0, len(cut_path.cut_list_1)):
            cut_lst_1 = copy.deepcopy(cut_path.cut_list_1[idx])
            cut_lst_2 = copy.deepcopy(cut_path.cut_list_2[idx])

            # If the cut_list is a SectionLink, add the two SectionLink points to the point list, else we normalize the
            # two CrossSections to a standardizes number of points and append those points to the point list.
            if isinstance(cut_lst_1, prim.SectionLink):
                path_1_tmp = cut_lst_1.get_path()
                path_2_tmp = cut_lst_2.get_path()
                logger.debug('SectionLink1: %s | SectionLink2: %s' % (path_1_tmp, path_2_tmp))
                path1w.extend(path_1_tmp)
                path2w.extend(path_2_tmp)
                if cut_lst_1.fast_cut:
                    speed_list.extend([wire_cutter.max_speed]*len(path_1_tmp))
                else:
                    speed_list.extend([wire_cutter.min_speed]*len(path_1_tmp))
            else:
                num_points = 256
                tmp_path_1 = prim.GeometricFunctions.normalize_path_points(cut_lst_1.get_path(), num_points=num_points)
                tmp_path_2 = prim.GeometricFunctions.normalize_path_points(cut_lst_2.get_path(), num_points=num_points)
                num_points = min(len(tmp_path_1), len(tmp_path_2))
                tmp_path_1 = tmp_path_1[0:num_points]
                tmp_path_2 = tmp_path_2[0:num_points]
                logger.debug('Normalized Path1 Len: %s | Normalized Path2 Len; %s' % (len(tmp_path_1), len(tmp_path_2)))
                path1w.extend(tmp_path_1)
                path2w.extend(tmp_path_2)
                speed_list.extend([wire_cutter.min_speed]*num_points)

        plt.figure()
        prim.GeometricFunctions.plot_path(path1w, 'k', scatter=False)
        prim.GeometricFunctions.plot_path(path2w, 'r', scatter=False)
        if output_dir is not None:
            plt.savefig(os.path.join(output_dir, 'tool_path.png'))
        else:
            plt.show()

        for idx in range(0, len(path1w)):
            line = prim.Line.line_from_points(path1w[idx], path2w[idx])
            logger.debug('*'*80)
            logger.debug('Point1: %s | Point2: %s' % (path1w[idx], path2w[idx]))
            gantry1_point = line.get_extrapolated_point(0, 'z')
            gantry2_point = line.get_extrapolated_point(wire_cutter.wire_length, 'z')
            if gantry1_point is None or gantry1_point is None:
                logger.debug('Invalid gantry point generated')
            logger.debug('Ex Point1: %s | Ex Point2: %s' % (gantry1_point, gantry2_point))
            logger.debug('Point 1 Diff: %s | Point 2 Diff: %s' % (gantry1_point-path1w[idx], gantry2_point-path2w[idx]))
            logger.debug('*'*80)

            path1.append(gantry1_point)
            path2.append(gantry2_point)

        file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'debug')
        prim.GeometricFunctions.path_to_csv(path1w, file_path=os.path.join(file_path, 'path1w.csv'))
        prim.GeometricFunctions.path_to_csv(path2w, file_path=os.path.join(file_path, 'path2w.csv'))
        prim.GeometricFunctions.path_to_csv(path1, file_path=os.path.join(file_path, 'path1.csv'))
        prim.GeometricFunctions.path_to_csv(path2, file_path=os.path.join(file_path, 'path2.csv'))

        return ToolPath(path1, path2, speed_list=speed_list)

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

    def get_relative_movement_list(self):
        movement = list()

        for idx in range(0, len(self._path1)-1):
            dx = self._path1[idx+1]['x'] - self._path1[idx]['x']
            dy = self._path1[idx+1]['y'] - self._path1[idx]['y']
            movement.append([dx, dy, 0, 0])

        for idx in range(0, len(self._path2)-1):
            du = self._path2[idx+1]['x'] - self._path2[idx]['x']
            dz = self._path2[idx+1]['y'] - self._path2[idx]['y']
            if idx < len(movement):
                movement[idx][2] = du
                movement[idx][3] = dz
            else:
                movement.append([0, 0, du, dz])
        return movement

    def get_absolute_movement_list(self):
        movement = list()

        for idx in range(0, len(self._path1)):
            movement.append([self._path1[idx]['x'], self._path1[idx]['y'], 0, 0])

        for idx in range(0, len(self._path2)-1):
            if idx < len(movement):
                movement[idx][2] = self._path2[idx]['x']
                movement[idx][3] = self._path2[idx]['y']
            else:
                movement.append([0, 0, self._path2[idx]['x'], self._path2[idx]['y']])
        return movement

    def animate(self, file_path=None, title=''):
        logger = logging.getLogger(__name__)

        fig, ax = plt.subplots()
        xdata1, ydata1 = [], []
        ln1, = plt.plot([], [], 'r')
        xdata2, ydata2 = [], []
        ln2, = plt.plot([], [], 'b')
        plt.title(title)
        plt.xlabel('X-U Axis (mm)')
        plt.xlabel('Y-Z Axis (mm)')

        def init():
            ax.set_xlim(0, 400)
            ax.set_ylim(-20, 400)
            return ln1, ln2,

        def update(frame):
            xdata1.append(self._path1[frame]['x'])
            ydata1.append(self._path1[frame]['y'])
            ln1.set_data(xdata1, ydata1)

            xdata2.append(self._path2[frame]['x'])
            ydata2.append(self._path2[frame]['y'])
            ln2.set_data(xdata2, ydata2)
            return ln1, ln2,

        # Set up formatting for the movie files

        ani = animation.FuncAnimation(fig, update, frames=list(range(0, len(self._path1))),
                            init_func=init, blit=True)
        if file_path is not None:
            try:
                plt.rcParams['animation.ffmpeg_path'] = r'C:\Program Files\ffmpeg-4.4\bin\ffmpeg.exe'
                writer_class = animation.writers['ffmpeg']
                writer = writer_class(fps=15, metadata=dict(artist='FurEter'), bitrate=1800)
                ani.save(file_path % title, writer=writer)
            except RuntimeError:
                logger.exception('Error animation writter could not be found at file path: %s',
                                 r'C:\Program Files\ffmpeg-4.4\bin\ffmpeg.exe')
        plt.show()
