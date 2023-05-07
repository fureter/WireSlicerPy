import abc
import copy
import csv
import logging
import os
import sys
from abc import ABC
from collections import namedtuple

import matplotlib.animation as animation
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
import numpy as np

import geometry.spatial_manipulation as sm

WorkPiece = namedtuple('WorkPiece', field_names=['width', 'height', 'thickness'])


# ----------------------------------------------------------------------------------------------------------------------
class Point(object):
    """

    """
    EQUAL_TOL = 1E-8

    def __init__(self, x, y, z):
        self._coord = np.array([x, y, z], dtype=float)

    def __getitem__(self, item):
        if isinstance(item, str):
            ret_val = {'x': self._coord[0],
                       'y': self._coord[1],
                       'z': self._coord[2]}[item]
        else:
            ret_val = self._coord[item]
        return ret_val

    def __setitem__(self, key, value):
        if isinstance(key, str):
            indx = {'x': 0,
                    'y': 1,
                    'z': 2}[key]
            self._coord[indx] = value

        else:
            self._coord[key] = value

    def __str__(self):
        return '(%s, %s, %s)' % (self._coord[0], self._coord[1], self._coord[2],)

    def __eq__(self, other):
        if isinstance(other, Point):
            return np.sqrt((other - self)**2) < self.EQUAL_TOL

    def __add__(self, other):
        if isinstance(other, Point):
            return Point(self._coord[0] + other['x'], self._coord[1] + other['y'], self._coord[2] + other['z'])
        elif isinstance(other, float) or isinstance(other, int):
            return Point(self._coord[0] + other, self._coord[1] + other, self._coord[2] + other)
        else:
            raise TypeError('Error: Point addition does not support Type: %s' % type(other))

    def __neg__(self):
        return Point(-self._coord[0], -self._coord[1], -self._coord[2])

    def __sub__(self, other):
        if isinstance(other, Point):
            return Point(self._coord[0] - other['x'], self._coord[1] - other['y'], self._coord[2] - other['z'])
        elif isinstance(other, float) or isinstance(other, int):
            return Point(self._coord[0] - other, self._coord[1] - other, self._coord[2] - other)
        else:
            raise TypeError('Error: Point subtraction does not support Type: %s' % type(other))

    def __pow__(self, power, modulo=None):
        return self._coord[0] ** power + self._coord[1] ** power + self._coord[2] ** power

    def __repr__(self):
        return '(%s, %s, %s)' % (self._coord[0], self._coord[1], self._coord[2],)

    def __rdiv__(self, other):
        """

        :param int or float other:
        :return:
        """
        return Point(self._coord[0]/other, self._coord[1]/other, self._coord[2]/other)

    def __truediv__(self, other):
        """

        :param int or float other:
        :return:
        """
        return Point(self._coord[0]/other, self._coord[1]/other, self._coord[2]/other)

    def __mul__(self, other):
        ret_val = None
        if isinstance(other, Point):
            ret_val = self._coord[0]*other['x'] + self._coord[1]*other['y'] + self._coord[2]*other['z']
        else:
            ret_val = Point(self._coord[0] * other, self._coord[1] * other,  self._coord[2] * other)
        return ret_val

    def as_ndarray(self):
        return np.array([self._coord[0], self._coord[1], self._coord[2]])


class Section(ABC):

    @abc.abstractmethod
    def get_path(self):
        raise NotImplementedError('Base class Section does not implement get_path')


class SectionLink(Section):
    """
    Used to define connections between CrossSections
    """
    def __init__(self, start_point, end_point, fast_cut=False):
        self.start_point = start_point
        self.end_point = end_point
        self.fast_cut = fast_cut

    def get_movement(self):
        return self.end_point - self.start_point

    def get_path(self):
        return [self.start_point, self.end_point]


class Line(Section):
    def __init__(self, a, b, c, x0, y0, z0, point1, point2, logger=None):
        self._a = float(a)
        self._b = float(b)
        self._c = float(c)
        self._x0 = float(x0)
        self._y0 = float(y0)
        self._z0 = float(z0)
        self._p1 = point1
        self._p2 = point2

        if logger is None:
            logger = logging.getLogger()
        self.logger = logger

    def get_extrapolated_point(self, constraint, constraint_dim):
        """
        Returns a point along the line at position `constraint` along the axis `constraint_dim`.

        :param float or int constraint: Position along the line to probe for a point.
        :param str or int constraint_dim: Axis relative to the line to test for a point.
        :return: Returns the point at `constraint` along the line at dimension `constraint_dim`, or returns
            None if the `constraint` and `constraint_dim` are not along the line.
        :rtype: None or Point.
        """
        ret_val = None
        x = None
        y = None
        z = None

        if constraint_dim == 'x':
            if self._a == 0.0:
                self.logger.warning('Warning: Point constraint is not along Line in the given dimension')
            else:
                val = (constraint - self._x0) / self._a

                x = constraint
                y = val * self._b + self._y0
                z = val * self._c + self._z0
        elif constraint_dim == 'y':
            if self._b == 0.0:
                self.logger.warning('Warning: Point constraint is not along Line in the given dimension')
            else:
                val = (constraint - self._y0) / self._b

                x = val * self._a + self._x0
                y = constraint
                z = val * self._c + self._z0
        elif constraint_dim == 'z':
            if self._c == 0.0:
                self.logger.warning('Warning: Point constraint is not along Line in the given dimension')
            else:
                val = (constraint - self._z0) / self._c

                x = val * self._a + self._x0
                y = val * self._b + self._y0
                z = constraint
        else:
            raise ValueError('Invalid constraint dim specified: %s, needs to be [x,y, or z]\r\n' % constraint_dim)
        if x is not None and y is not None and z is not None:
            ret_val = Point(x, y, z)

        return ret_val

    def get_path(self):
        """
        Returns the two points (x0, y0, z0) and (a + x0, b + y0, c + z0) used to form the Line.
        :return: Two points used to form the Line.
        """
        x1 = self._a + self._x0
        y1 = self._b + self._y0
        z1 = self._c + self._z0
        return [Point(self._x0, self._y0, self._z0), Point(x1, y1, z1)]

    @staticmethod
    def line_from_points(point1, point2):
        """
        Takes two points and forms a line in the form of:
        (x-x0)/a = (y-y0)/b = (z-z0)/c

        :param Point point1: Origin starting point. point1 is used to obtain x0, y0, z0
        :param Point point2: Offset point from the origin, used to find a, b, c.
        """
        logger = logging.getLogger(__name__)
        if point1 is None or point2 is None:
            logger.debug('Something is very wrong')
        x0 = point1['x']
        y0 = point1['y']
        z0 = point1['z']

        a = point2['x'] - x0
        b = point2['y'] - y0
        c = point2['z'] - z0

        return Line(a, b, c, x0, y0, z0, point1, point2)

    def intersects(self, line):
        """
        returns true if the given line intersects with self
        :param line:
        :return:
        """
        logger = logging.getLogger(__name__)

        ret_val = False

        val = None
        x, y, z = None, None, None
        if not self.parallel(line):
            try:
                sx, sy = line._a, line._b
                rx, ry = self._a, self._b

                a = np.array([[rx, -sx],
                              [ry, -sy]])
                b = np.array([[-self._x0 + line._x0], [-self._y0 + line._y0]])
                val = np.linalg.solve(a=a, b=b)
            except np.linalg.LinAlgError:
                return False, None

            x = val[0] * self._a + self._x0
            y = val[0] * self._b + self._y0
            z = val[0] * self._c + self._z0

            p_l1 = self.get_path()
            p_l2 = line.get_path()

            min_x_l1 = min([p_l1[0]['x'], p_l1[1]['x']])
            max_x_l1 = max([p_l1[0]['x'], p_l1[1]['x']])
            min_x_l2 = min([p_l2[0]['x'], p_l2[1]['x']])
            max_x_l2 = max([p_l2[0]['x'], p_l2[1]['x']])

            min_y_l1 = min([p_l1[0]['y'], p_l1[1]['y']])
            max_y_l1 = max([p_l1[0]['y'], p_l1[1]['y']])
            min_y_l2 = min([p_l2[0]['y'], p_l2[1]['y']])
            max_y_l2 = max([p_l2[0]['y'], p_l2[1]['y']])

            if ((min_x_l1 <= x <= max_x_l1) and (min_x_l2 <= x <= max_x_l2) and
                    (min_y_l1 <= y <= max_y_l1) and (min_y_l2 <= y <= max_y_l2)):
                # In most cases where we use intersection, everything will be in the z-plane, therefore we check <=
                # for the z-axis
                if (p_l1[0]['z'] <= z <= p_l1[1]['z'] or p_l1[1]['z'] <= z <= p_l1[0]['z']) and\
                        (p_l2[0]['z'] <= z <= p_l2[1]['z'] or p_l2[1]['z'] <= z <= p_l2[0]['z']):
                    ret_val = True
                #logger.debug('Val 1: %s, Val 2: %s', val[0], val[1])
                #logger.debug('x_int: %s, y_int: %s, z_int: %s', x, y, z)
                #logger.debug('x_l1: %s, y_l1: %s, z_l1: %s', self._x0, self._y0, self._z0)

        return ret_val, copy.deepcopy(Point(x, y, z))

    def signed_distance_to_point_xy(self, point):
        """

        :param Point point:
        :return:
        """

        x1 = self._a + self._x0
        y1 = self._b + self._y0

        return (point['x']-self._x0)*(y1 - self._y0) - (point['y'] - self._y0)*(x1 - self._x0)

    def coord_in_range_dim(self, coord, dim):
        ret_val = False
        if self._p1[dim] <= coord <= self._p2[dim]:
            ret_val = True
        if self._p2[dim] <= coord <= self._p1[dim]:
            ret_val = True
        return ret_val

    def normal(self):
        return np.array([-self._b, self._a, 0]) / np.sqrt(self._b**2 + self._a**2)

    def parallel(self, line):
        counter = 0

        if self._a == line._a:
            counter += 1
        if self._b == line._b:
            counter += 1
        if self._c == line._c:
            counter += 1

        ret_val = True if counter > 1 else False

        return ret_val

    def plot(self):  # pragma: no cover
        x1 = self._a + self._x0
        y1 = self._b + self._y0

        plt.plot([self._x0, x1], [self._y0, y1])


class Path(Section):
    def __init__(self, points):
        self.path = points

    def get_path(self):
        return self.path


# ----------------------------------------------------------------------------------------------------------------------
class Plane(object):
    def __init__(self, x0, y0, z0, a, b, c):
        self._x0 = x0
        self._y0 = y0
        self._z0 = z0
        self._a = a
        self._b = b
        self._c = c

    @staticmethod
    def plane_from_3_points(point1, point2, point3):
        a = np.array([[point1['x'], point1['y'], point1['z']],
                      [point2['x'], point2['y'], point2['z']],
                      [point3['x'], point3['y'], point3['z']]])
        b = np.array([0, 0, 0])

        (a, b, c) = np.linalg.solve(a, b)
        return Plane(x0=point1['x'], y0=point1['y'], z0=point1['z'], a=a, b=b, c=c)

    @staticmethod
    def plane_from_point_norm(point, norm):
        """
        :param point:
        :param norm:
        :return:
        """
        return Plane(x0=point['x'], y0=point['y'], z0=point['z'], a=norm[0], b=norm[1], c=norm[2])

    @staticmethod
    def plane_from_point_2_vec(point, vec1, vec2):
        vec = np.cross(vec1, vec2)
        return Plane.plane_from_point_norm(point, vec)

    @property
    def origin(self):
        return [self._x0, self._y0, self._z0]

    @property
    def normal(self):
        return [self._a, self._b, self._c]


# ----------------------------------------------------------------------------------------------------------------------
class Spline(Section):
    def __init__(self, points, closed_loop=True, logger=None, resolution=2):
        if logger is None:
            logger = logging.getLogger()
        self.logger = logger

        self.resolution = resolution

        self._points = points
        if closed_loop:
            # If the spline is a closed loop, duplicate the initial point and place it at the end of the loop. This
            # allows the cubic spline to have a segment connecting the last point and the inital point.
            self._points = np.append(self._points, points[0])
        self._loop = closed_loop
        self.segments = len(self._points) - 1

        self.ax = np.zeros(self.segments)
        self.bx = np.zeros(self.segments)
        self.cx = np.zeros(self.segments)
        self.dx = np.zeros(self.segments)

        self.ay = np.zeros(self.segments)
        self.by = np.zeros(self.segments)
        self.cy = np.zeros(self.segments)
        self.dy = np.zeros(self.segments)

        self.az = np.zeros(self.segments)
        self.bz = np.zeros(self.segments)
        self.cz = np.zeros(self.segments)
        self.dz = np.zeros(self.segments)

        self._calculate_coefficients()

    def _calculate_coefficients(self):
        # loop through the 3 dimensions to get coefficients for x, y, and z
        for i in range(0, 3):
            num_points = len(self._points)
            mat = np.zeros((4 * self.segments, 4 * self.segments))
            var = np.zeros(4 * self.segments)

            cnt = 0
            for n in range(0, num_points - 1):
                inx = 4 * n
                inx_prev = 4 * (n - 1)

                mat[inx, inx] = (n + 1) ** 3
                mat[inx, inx + 1] = (n + 1) ** 2
                mat[inx, inx + 2] = (n + 1) ** 1
                mat[inx, inx + 3] = 1

                mat[inx + 1, inx] = (n + 2) ** 3
                mat[inx + 1, inx + 1] = (n + 2) ** 2
                mat[inx + 1, inx + 2] = (n + 2) ** 1
                mat[inx + 1, inx + 3] = 1

                if n > 0:
                    mat[inx + 2, inx_prev] = 3 * (n + 1) ** 2
                    mat[inx + 2, inx_prev + 1] = 2 * (n + 1) ** 1
                    mat[inx + 2, inx_prev + 2] = 1

                    mat[inx + 2, inx] = -3 * (n + 1) ** 2
                    mat[inx + 2, inx + 1] = -2 * (n + 1) ** 1
                    mat[inx + 2, inx + 2] = -1

                    mat[inx + 3, inx_prev] = 6 * (n + 1) ** 1
                    mat[inx + 3, inx_prev + 1] = 2

                    mat[inx + 3, inx] = -6 * (n + 1) ** 1
                    mat[inx + 3, inx + 1] = -2

                var[inx] = self._points[n][i]
                var[inx + 1] = self._points[n + 1][i]

            mat[2, 0] = 6
            mat[2, 1] = 2

            mat[3, 4 * self.segments - 3] = 6 * num_points
            mat[3, 4 * self.segments - 2] = 2
            i_coeff = np.linalg.solve(mat, var)

            for n in range(0, self.segments):
                if i == 0:
                    self.ax[n] = i_coeff[4 * n]
                    self.bx[n] = i_coeff[4 * n + 1]
                    self.cx[n] = i_coeff[4 * n + 2]
                    self.dx[n] = i_coeff[4 * n + 3]
                elif i == 1:
                    self.ay[n] = i_coeff[4 * n]
                    self.by[n] = i_coeff[4 * n + 1]
                    self.cy[n] = i_coeff[4 * n + 2]
                    self.dy[n] = i_coeff[4 * n + 3]
                else:
                    self.az[n] = i_coeff[4 * n]
                    self.bz[n] = i_coeff[4 * n + 1]
                    self.cz[n] = i_coeff[4 * n + 2]
                    self.dz[n] = i_coeff[4 * n + 3]

    def get_spline_length(self):
        """

        :return:
        """

        dt = 1
        t = 0 + dt
        length = 0

        while t < self.segments+1:
            pos = t
            pos_m = t - dt
            indx = self._get_segment(pos)
            indx_m = self._get_segment(pos_m)
            x = self._get_x(pos, indx)
            y = self._get_y(pos, indx)
            z = self._get_z(pos, indx)
            x_m = self._get_x(pos_m, indx_m)
            y_m = self._get_y(pos_m, indx_m)
            z_m = self._get_z(pos_m, indx_m)
            length += np.sqrt((x - x_m) ** 2 + (y - y_m) ** 2 + (z - z_m) ** 2)
            t += dt

        return length

    def get_point_from_distance(self, dist, t=1E-1, dt=1E-1, start=0):
        """

        :param dist:
        :return:
        """
        x = 0
        y = 0
        z = 0
        curr_seg = t
        if t < dt:
            curr_seg = dt
        length = start

        while length <= dist and curr_seg < self.segments:
            # self.logger.info('\tcurrently at distance %smm out of %smm with segment: %s out of %s' %
            #                  (length, dist, curr_seg, self.segments))
            pos = curr_seg
            pos_m = curr_seg - dt
            indx = self._get_segment(pos)
            indx_m = self._get_segment(pos_m)
            x = self._get_x(pos, indx)
            y = self._get_y(pos, indx)
            z = self._get_z(pos, indx)
            x_m = self._get_x(pos_m, indx_m)
            y_m = self._get_y(pos_m, indx_m)
            z_m = self._get_z(pos_m, indx_m)
            dlen = np.sqrt((x - x_m) ** 2 + (y - y_m) ** 2 + (z - z_m) ** 2)
            length += dlen
            curr_seg += dt

        point = Point(x, y, z)

        return point, curr_seg, length

    def get_x_y_z(self, resolution=None):
        """

        :param resolution:
        :return:
        """
        if resolution is None:
            resolution = self.resolution

        x = np.zeros(self.segments * resolution + 1)
        y = np.zeros(self.segments * resolution + 1)
        z = np.zeros(self.segments * resolution + 1)

        for i in range(0, len(x)):
            pos = i / float(resolution)
            indx = self._get_segment(pos)
            pos = pos + 1
            x[i] = self._get_x(pos, indx)
            y[i] = self._get_y(pos, indx)
            z[i] = self._get_z(pos, indx)

        return x, y, z

    def get_points(self, resolution=None):
        if resolution is None:
            resolution = self.resolution

        x, y, z = self.get_x_y_z(resolution=resolution)

        points = list()
        for i in range(0, len(x)):
            points.append(Point(x[i], y[i], z[i]))

        return points

    def get_path(self):
        return self.get_points(resolution=self.resolution)

    def plot_spline(self):
        (x, y, z) = self.get_x_y_z()

        plt.plot(x, y)
        plt.draw()

    def _get_x(self, pos, indx):
        return self.ax[indx] * pos ** 3 + self.bx[indx] * pos ** 2 + self.cx[indx] * pos + self.dx[indx]

    def _get_y(self, pos, indx):
        return self.ay[indx] * pos ** 3 + self.by[indx] * pos ** 2 + self.cy[indx] * pos + self.dy[indx]

    def _get_z(self, pos, indx):
        return self.az[indx] * pos ** 3 + self.bz[indx] * pos ** 2 + self.cz[indx] * pos + self.dz[indx]

    @staticmethod
    def _write_numerical_to_csv(mat, var, i_coeff, file_name, directory):
        """

        :param mat:
        :param var:
        :param i_coeff:
        :param file_name:
        :param directory:
        :return:
        """

        def _field_names(num_col):
            field = list()
            for ind in range(0, num_col):
                field.append('A%s' % (ind + 1))
            field.append('x')
            field.append('b')
            return field

        with open(os.path.join(directory, file_name), 'w', newline='') as csv_file:
            n = len(var)
            writer = csv.DictWriter(csv_file, _field_names(n))
            writer.writeheader()

            for i in range(0, n):
                vals = dict()

                for j in range(0, n):
                    vals['A%s' % (j + 1)] = mat[i, j]
                vals['x'] = i_coeff[i]
                vals['b'] = var[i]
                writer.writerow(vals)

    def _get_segment(self, t):
        for i in range(0, self.segments):
            if i <= t <= i + 1:
                return i
        if t == self.segments + 1:
            return self.segments

        raise AttributeError('T(%s) out of range for the given intervals(%s)' % (t, self.segments))


class GeometricFunctions(object):
    @staticmethod
    def plot_potential_field(x_mesh, y_mesh, potential_field, output_dir, index):
        plt.figure(figsize=(16, 9), dpi=160)
        plt.pcolormesh(x_mesh, y_mesh, np.log10(np.sqrt(potential_field[:, :, 0]**2 + potential_field[:, :, 1]**2)),
                       shading='auto')
        bar = plt.colorbar()
        bar.label = 'Potential Magnitude'
        plt.quiver(x_mesh, y_mesh, potential_field[:, :, 0], potential_field[:, :, 1], width=0.001)
        plt.title('Potential Field with Force Vectors')
        plt.savefig(os.path.join(output_dir, 'potential_field_w_quiver_%s.png' % index))
        plt.figure(figsize=(16, 9), dpi=160)
        plt.pcolormesh(x_mesh, y_mesh, np.log10(np.sqrt(potential_field[:, :, 0]**2 + potential_field[:, :, 1]**2)),
                       shading='auto')
        plt.title('Potential Field')
        bar = plt.colorbar()
        bar.label = 'Potential Magnitude'
        plt.savefig(os.path.join(output_dir, 'potential_field_%s.png' % index))

    @staticmethod
    def plot_cross_sections_on_workpiece(state_dict, work_piece, output_dir, num_sections, index=0):
        """

        :param state_dict:
        :param WorkPiece work_piece:
        :return:
        """
        x = list()
        y = list()
        plt.close('all')
        plt.figure(figsize=(16, 9), dpi=320)
        plt.plot([0, 0, work_piece.width, work_piece.width, 0], [0, work_piece.height, work_piece.height, 0, 0],
                 'g')

        for ind in range(0, len(state_dict)):
            if ind in state_dict and state_dict[ind]['cut'] == num_sections:
                path_1 = copy.deepcopy(state_dict[ind]['section'].section1.get_path())
                path_2 = copy.deepcopy(state_dict[ind]['section'].section2.get_path())
                collider_path1 = copy.deepcopy(state_dict[ind]['collider1'].get_path())
                collider_path2 = copy.deepcopy(state_dict[ind]['collider2'].get_path())

                GeometricFunctions.move_cross_section_from_state_dict(
                    path_1=path_1,
                    path_2=path_2,
                    dict_entry=state_dict[ind])

                bb1 = GeometricFunctions.get_bounding_box_from_path(collider_path1 + collider_path2)

                plt.plot([bb1[0][0], bb1[0][0], bb1[1][0], bb1[1][0], bb1[0][0]],
                         [bb1[0][1], bb1[1][1], bb1[1][1], bb1[0][1], bb1[0][1]], 'k')

                GeometricFunctions.plot_path(path_1, 'r', scatter=False)
                GeometricFunctions.plot_path(path_2, 'b', scatter=False)
                GeometricFunctions.plot_path(collider_path1, 'm', scatter=False)
                GeometricFunctions.plot_path(collider_path2, 'c', scatter=False)
                x.append(state_dict[ind]['x_pos'])
                y.append(state_dict[ind]['y_pos'])

                txt = plt.text(state_dict[ind]['x_pos'], state_dict[ind]['y_pos'], s='C%s' % (ind + 1),
                               fontsize='medium')
                txt.set_path_effects([pe.withStroke(linewidth=3, foreground='w')])
            plt.axis('equal')
        plt.savefig(os.path.join(output_dir, 'Cross_Section_pos_sheet%s_%s.png' % (num_sections, index)))

    @staticmethod
    def bounding_boxes_intersect(bb1, bb2):
        ret_val = False
        if (((bb1[0][0] <= bb2[0][0] <= bb1[1][0] or bb1[0][0] <= bb2[1][0] <= bb1[1][0]) and
                (bb1[0][1] <= bb2[0][1] <= bb1[1][1] or bb1[0][1] <= bb2[1][1] <= bb1[1][1]))
                or ((bb2[0][0] <= bb1[0][0] <= bb2[1][0] or bb2[0][0] <= bb1[1][0] <= bb2[1][0]) and
                    (bb2[0][1] <= bb1[0][1] <= bb2[1][1] or bb2[0][1] <= bb1[1][1] <= bb2[1][1]))):
            ret_val = True
        return ret_val

    @staticmethod
    def move_cross_section_from_state_dict(path_1, path_2, dict_entry):
        center = Point(0, 0, 0)
        for point in path_1:
            center += point
        for point in path_2:
            center += point
        center /= (len(path_1) + len(path_2))
        sm.PointManip.Transform.translate(path_1, -center)
        sm.PointManip.Transform.translate(path_2, -center)

        sm.PointManip.Transform.rotate(path_1, [0, 0, np.deg2rad(dict_entry['rot'])])
        sm.PointManip.Transform.rotate(path_2, [0, 0, np.deg2rad(dict_entry['rot'])])

        sm.PointManip.Transform.translate(path_1, [dict_entry['x_pos'], dict_entry['y_pos'], 0])
        sm.PointManip.Transform.translate(path_2, [dict_entry['x_pos'], dict_entry['y_pos'], 0])

    @staticmethod
    def get_bounding_box_from_path(path):
        min_x_p, max_x_p = GeometricFunctions.get_point_from_min_coord(path, 'x')['x'], \
                           GeometricFunctions.get_point_from_max_coord(path, 'x')[1]

        min_y_p, max_y_p = GeometricFunctions.get_point_from_min_coord(path, 'y')['y'], \
                           GeometricFunctions.get_point_from_max_coord(path, 'y')[1]

        min_z_p, max_z_p = GeometricFunctions.get_point_from_min_coord(path, 'z')['z'], \
                           GeometricFunctions.get_point_from_max_coord(path, 'z')[1]

        return [Point(min_x_p, min_y_p, min_z_p), Point(max_x_p, max_y_p, max_z_p)]

    @staticmethod
    def center_path(path):
        center = GeometricFunctions.get_center_of_path(path)
        sm.PointManip.Transform.translate(path, [-center['x'], -center['y'], -center['z']])

    @staticmethod
    def get_center_of_path(path):
        center = Point(0, 0, 0)
        for point in path:
            center += point
        center /= len(path)
        return center

    @staticmethod
    def get_point_from_max_coord(path, dim):
        if not isinstance(path[0], Point):
            if dim == 'x':
                dim = 0
            elif dim == 'y':
                dim = 1
            elif dim == 'z':
                dim = 2
        max = -sys.maxsize - 1
        point = None
        for idx in range(0, len(path)):
            if path[idx][dim] > max:
                max = path[idx][dim]
                point = path[idx]

        return point, max

    @staticmethod
    def get_point_from_min_coord(path, dim):
        if not isinstance(path[0], Point):
            if dim == 'x':
                dim = 0
            elif dim == 'y':
                dim = 1
            elif dim == 'z':
                dim = 2
        min = sys.maxsize
        point = None
        for idx in range(0, len(path)):
            if path[idx][dim] < min:
                min = path[idx][dim]
                point = path[idx]

        return point

    @staticmethod
    def get_index_max_coord(path, dim):
        if not isinstance(path[0], Point):
            if dim == 'x':
                dim = 0
            elif dim == 'y':
                dim = 1
            elif dim == 'z':
                dim = 2
        max = -sys.maxsize - 1
        indx = 0
        for idx in range(0, len(path)):
            if path[idx][dim] > max:
                max = path[idx][dim]
                indx = idx

        return indx

    @staticmethod
    def get_index_min_coord(path, dim):
        if not isinstance(path[0], Point):
            if dim == 'x':
                dim = 0
            elif dim == 'y':
                dim = 1
            elif dim == 'z':
                dim = 2
        min = sys.maxsize
        indx = 0
        for idx in range(0, len(path)):
            if path[idx][dim] < min:
                min = path[idx][dim]
                indx = idx

        return indx

    @staticmethod
    def get_multi_index_min_coord(path, dim, num_return):
        if not isinstance(path[0], Point):
            if dim == 'x':
                dim = 0
            elif dim == 'y':
                dim = 1
            elif dim == 'z':
                dim = 2
        indx = list()
        for ind in range(num_return):
            tmp_idx = 0
            min = sys.maxsize
            for idx in range(0, len(path)):
                if path[idx][dim] < min and idx not in indx:
                    min = path[idx][dim]
                    tmp_idx = idx
            indx.append(tmp_idx)

        return indx

    @staticmethod
    def path_length(path):
        length = 0
        if isinstance(path, list) or isinstance(path, np.ndarray):
            for idx in range(0, len(path) - 1):
                diff = path[idx + 1] - path[idx]
                length += np.sqrt(diff ** 2)
        else:
            raise NotImplementedError('Error: path_length is currently only implemented for a list of points defining'
                                      ' the path')
        return length

    @staticmethod
    def path_length_from_point_to_point(path, point_1, point_2):
        length = 0
        valid_region = False
        if isinstance(path, list):
            for idx in range(0, len(path) - 1):
                if path[idx] == point_1:
                    valid_region = True
                if valid_region:
                    diff = path[idx + 1] - path[idx]
                    length += np.sqrt(diff ** 2)
                if path[idx + 1] == point_2:
                    valid_region = False
            if length == 0:
                print(point_1)
                raise ValueError('Error: Key Point 1: %s Point 2: %s was never found on the path with start point: %s' %
                                 (point_1, point_2, path[0]))
        else:
            raise NotImplementedError('Error: path_length is currently only implemented for a list of points defining'
                                      ' the path')
        return length

    @staticmethod
    def get_points_with_chord_offset(path, chord_offset):
        ret_points = list()
        dist = list()
        if len(path) > 2:
            ret_points.append(path[0])
            ret_points.append(path[1])
            dist.append(abs(chord_offset - path[0]['x']))
            dist.append(abs(chord_offset - path[1]['x']))
            for ind in range(2, len(path)):
                curr_dist = abs(chord_offset - path[ind]['x'])
                if dist[0] > curr_dist:
                    ret_points[1] = ret_points[0]
                    ret_points[0] = path[ind]
                    dist[1] = dist[0]
                    dist[0] = curr_dist
                elif dist[1] > curr_dist:
                    ret_points[1] = path[ind]
                    dist[1] = curr_dist
        return ret_points

    @staticmethod
    def get_point_along_path(path, distance, start_point=None):
        length = 0
        index = 0

        valid_region = True if start_point is None else False

        if isinstance(path, list) or isinstance(path, np.ndarray):
            for idx in range(0, len(path) - 1):
                # If a start point was provided, check if we have reached the start point and set valid_region to true
                if start_point is not None and path[idx] == start_point:
                    valid_region = True

                # Only calculate length if the points are within a valid region
                if valid_region:
                    length += np.sqrt((path[idx + 1] - path[idx]) ** 2)
                    # If we have travelled the desired distance break out
                    if distance <= length:
                        index = idx + 1
                        break
            if length == 0:
                raise ValueError('Error: Start point: %s was never found on the path' % start_point)

            prev_length = length - np.sqrt((path[index] - path[index - 1]) ** 2)
            diff = length - prev_length
            if diff == 0:
                return path[-1]
            ratio = (distance - prev_length) / diff
            x = (path[index]['x'] - path[index - 1]['x']) * ratio + path[index - 1]['x']
            y = (path[index]['y'] - path[index - 1]['y']) * ratio + path[index - 1]['y']
            z = (path[index]['z'] - path[index - 1]['z']) * ratio + path[index - 1]['z']
            point = Point(x, y, z)

        else:
            raise NotImplementedError('Error: get_point_along_path currently only implemented for a list'
                                      ' of points defining the path')
        return point

    @staticmethod
    def close_path(path):
        """
        Assumes path has been sorted.
        :param path:
        :return:
        """
        if path[0] != path[-1]:
            if isinstance(path, list):
                path.append(copy.deepcopy(path[0]))
            elif isinstance(path, np.ndarray):
                path = np.append(path, [copy.deepcopy(path[0])])
        return path

    @staticmethod
    def remove_duplicate_memory_from_path(path):
        """

        :param list[Point] path:
        :return:
        :rtype: list[Point]
        """
        logger = logging.getLogger(__name__)
        addr_list = list()
        tmp_path = list()
        for point in path:
            addr = hex(id(point))
            if addr not in addr_list:
                addr_list.append(addr)
                tmp_path.append(point)
            else:
                logger.debug('Point with memory Address(%s) removed from path' % addr)
        return tmp_path

    @staticmethod
    def get_max_thickness(path, dim, ref_dim):
        """

        :param path:
        :param dim:
        :param ref_dim:
        :return:
        """
        thickness = None

        dim = GeometricFunctions._transform_dim(path[0], dim)

        # Get the maximum point along the dimension we are using as a reference.
        max_point = GeometricFunctions.get_point_from_max_coord(path, ref_dim)
        min_point = GeometricFunctions.get_point_from_min_coord(path, ref_dim)
        chord = max_point[ref_dim] - min_point[ref_dim]

        scale = 100 if chord < 10 else 10

        # Subdivide the reference dimension into additional segments for increased resolution. Scale is based off of a
        # loose estimation of the segments units.
        for idx in range(0, int(chord*scale)):
            points = GeometricFunctions.get_points_closest_to_line(path, Line())

        return thickness

    @staticmethod
    def get_points_closest_to_line(path, line):
        pass

    @staticmethod
    def normalize_path_points(path, num_points):
        """

        :param list[Point] path:
        :param int or float num_points:
        :return:
        """
        path_length = GeometricFunctions.path_length(path)

        path1 = list()

        path1.append(path[0])

        delta_dist = path_length / num_points
        curr_dist = delta_dist
        while curr_dist < path_length:
            point = GeometricFunctions.get_point_along_path(path, curr_dist)
            path1.append(point)
            curr_dist += delta_dist


        # GeometricFunctions.plot_path(path1, color='g')
        # plt.title('normalized_path')
        # plt.show()

        return path1

    @staticmethod
    def path_to_csv(path, file_path):
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        with open(file_path, 'w', newline='') as csv_file:
            field_names = ['index', 'x', 'y', 'z']
            writer = csv.DictWriter(csv_file, fieldnames=field_names)
            writer.writeheader()
            for index in range(0, len(path)):
                writer.writerow({
                    'index': index,
                    'x': path[index]['x'],
                    'y': path[index]['y'],
                    'z': path[index]['z']
                })

    @staticmethod
    def offset_curve(path, offset_scale, dir, divisions, add_leading_edge=True):
        logger = logging.getLogger(__name__)
        original_leading_edge = copy.deepcopy(path[0])
        original_curve = copy.deepcopy(path)
        for ind in range(0, divisions):
            path = GeometricFunctions.parallel_curve(path, offset_scale / divisions, dir)
        #     num_intersections = GeometricFunctions.number_of_intersections_in_path(path)
        #     logger.debug('Found %s intersections in path', num_intersections)
        #

        path = GeometricFunctions.clean_intersections(path, original_curve, offset_scale)

        if add_leading_edge:
            intersections = GeometricFunctions.get_all_intersection_points(path)
            if dir == 0:
                path.insert(0, original_leading_edge + Point(-offset_scale * [1, -1][dir], 0, 0))
                path.append(original_leading_edge + Point(-offset_scale * [1, -1][dir], 0, 0))
            else:
                closest_point = None
                min_dist = sys.maxsize
                for intersection in intersections:
                    dist = np.sqrt((path[0] - intersection)**2)
                    if dist < min_dist:
                        closest_point = intersection
                        min_dist = dist
                if closest_point is not None:
                    path.insert(0, copy.copy(closest_point))
                    path.append(copy.copy(closest_point))

        return path

    @staticmethod
    def parallel_curve(path, offset_scale, direction, plot_debug=False):
        logger = logging.getLogger(__name__)
        offset_path = list()

        center = copy.deepcopy(path[0])
        for ind in range(1, len(path)):
            center += path[ind]
        center /= len(path)

        norm1, norm2 = GeometricFunctions.normal_vector(path[-1], path[0], path[1])
        if plot_debug:
            plt.plot([path[0]['x'], path[0]['x'] + norm1['x'] * offset_scale],
                     [path[0]['y'], path[0]['y'] + norm1['y'] * offset_scale],
                     'k-')
            plt.plot([path[0]['x'], path[0]['x'] + norm2['x'] * offset_scale],
                     [path[0]['y'], path[0]['y'] + norm2['y'] * offset_scale],
                     'y-')
        vec = path[0] - center
        norm = norm1 if direction == 0 else norm2
        offset_path.append(path[0] + Point(norm['x'] * offset_scale, norm['y'] * offset_scale, 0))

        for ind in range(1, len(path)-1):
            norm1, norm2 = GeometricFunctions.normal_vector(path[ind-1], path[ind], path[ind+1])
            if plot_debug:
                plt.plot([path[ind]['x'], path[ind]['x'] + norm1['x'] * offset_scale],
                         [path[ind]['y'], path[ind]['y'] + norm1['y'] * offset_scale],
                         'k-')
                plt.plot([path[ind]['x'], path[ind]['x'] + norm2['x'] * offset_scale],
                         [path[ind]['y'], path[ind]['y'] + norm2['y'] * offset_scale],
                         'y-')
            norm = norm1 if direction == 0 else norm2
            offset_path.append(path[ind] + Point(norm['x'] * offset_scale, norm['y'] * offset_scale, 0))

        norm1, norm2 = GeometricFunctions.normal_vector(path[-2], path[-1], path[0])
        if plot_debug:
            plt.plot([path[-1]['x'], path[-1]['x'] + norm1['x'] * offset_scale],
                     [path[-1]['y'], path[-1]['y'] + norm1['y'] * offset_scale],
                     'k-')
            plt.plot([path[-1]['x'], path[-1]['x'] + norm2['x'] * offset_scale],
                     [path[-1]['y'], path[-1]['y'] + norm2['y'] * offset_scale],
                     'y-')
        vec = path[-1] - center
        norm = norm1 if direction == 0 else norm2
        offset_path.append(path[-1] + Point(norm['x'] * offset_scale, norm['y'] * offset_scale, 0))
        #offset_path.append(path[0] + Point(-offset_scale, 0, 0))

        return offset_path

    @staticmethod
    def clean_intersections(path, original_path, offset_amount, debug=False):
        logger = logging.getLogger(__name__)
        cleaned_path = list()
        deleted_ponits = list()

        for ind in range(0, len(path)):
            keep = True
            for ind2 in range(0, len(original_path)):
                dist = np.sqrt((path[ind] - original_path[ind2])**2)
                if dist < offset_amount*0.9999999999:
                    keep = False
            if keep:
                cleaned_path.append(path[ind])
            else:
                deleted_ponits.append(path[ind])
        if debug:
            plt.figure()
            GeometricFunctions.plot_path(original_path, 'C18', True, 'C19')
            GeometricFunctions.plot_path(cleaned_path, 'C20', True, 'C21')
            GeometricFunctions.plot_path(deleted_ponits, 'C22', True, 'C23')
            plt.legend(['Original', 'Cleaned', 'Deleted','Original_Points','Cleaned_Points', 'Deleted_Points'])
            plt.show()

        return cleaned_path

    @staticmethod
    def get_all_intersection_points(path):
        ret_val = list()
        for ind in range(0, len(path)-1):
            line = Line.line_from_points(path[ind], path[ind+1])
            for ind2 in range(ind+1, len(path)-1):
                if ind != ind2:
                    line2 = Line.line_from_points(path[ind2], path[ind2+1])
                    intersected, point = line.intersects(line2)
                    if intersected:
                        ret_val.append(point)
        return ret_val

    @staticmethod
    def number_of_intersections_in_path(path):
        ret_val = 0
        for ind in range(0, len(path)-1):
            line = Line.line_from_points(path[ind], path[ind+1])
            for ind2 in range(ind+1, len(path)-1):
                if ind != ind2:
                    line2 = Line.line_from_points(path[ind2], path[ind2+1])
                    intersected, point = line.intersects(line2)
                    if intersected:
                        ret_val += 1
        return ret_val

    @staticmethod
    def normal_vector(prev_point, curr_point, next_point):
        """

        :param Point prev_point:
        :param Point curr_point:
        :param next_point:
        :type next_point: Point or None
        :return:
        """
        diff_back = curr_point - prev_point
        if next_point is not None:
            diff_forw = next_point - curr_point

            norm_1 = (Point(-diff_back['y'], diff_back['x'], 0) + Point(-diff_forw['y'], diff_forw['x'], 0))/2.0
            norm_2 = (Point(diff_back['y'], -diff_back['x'], 0) + Point(diff_forw['y'], -diff_forw['x'], 0))/2.0
        else:

            norm_1 = (Point(-diff_back['y'], diff_back['x'], 0))
            norm_2 = (Point(diff_back['y'], -diff_back['x'], 0))

        return norm_1/np.sqrt(norm_1**2), norm_2/np.sqrt(norm_2**2)

    @staticmethod
    def plot_path(path, color, scatter=True, scatter_color=None, scatter_size=3):
        len_path = len(path)

        x = np.zeros(len_path)
        y = np.zeros(len_path)

        for i in range(0, len_path):
            x[i] = path[i]['x']
            y[i] = path[i]['y']

        if color is not None:
            plt.plot(x, y, color)
        else:
            plt.plot(x, y)
        if scatter:
            if scatter_color is not None:
                plt.scatter(x, y, c=scatter_color, s=scatter_size)
            else:
                plt.scatter(x, y, s=scatter_size)

    @staticmethod
    def animate_path(path):
        fig, ax = plt.subplots()
        xdata1, ydata1 = [], []
        ln1, = plt.plot([], [], 'r')

        def init():
            ax.set_xlim(-400, 400)
            ax.set_ylim(-400, 400)
            return ln1,

        def update(frame):
            xdata1.append(path[frame]['x'])
            ydata1.append(path[frame]['y'])
            ln1.set_data(xdata1, ydata1)

            return ln1,

        ani = animation.FuncAnimation(fig, update, frames=list(range(0, len(path))),
                                      init_func=init, blit=True)
        plt.show()

    @staticmethod
    def _transform_dim(item, dim):
        if not isinstance(item, Point):
            if dim == 'x':
                dim = 0
            elif dim == 'y':
                dim = 1
            elif dim == 'z':
                dim = 2
        return dim
