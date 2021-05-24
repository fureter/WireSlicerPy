import sys
import os
import csv
import logging

import numpy as np
import matplotlib.pyplot as plt


# ----------------------------------------------------------------------------------------------------------------------
class Point():
    """

    """

    def __init__(self, x, y, z):
        self._coord = np.array([x, y, z], dtype=np.float)

    def __get__(self, instance, owner):
        return self._coord

    def __set__(self, instance, value):
        self._coord = value

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
            return (other['x'] == self['x']) and (other['y'] == self['y']) and (other['z'] == self['z'])

    def __add__(self, other):
        if isinstance(other, Point):
            return Point(self._coord[0] + other['x'], self._coord[1] + other['y'], self._coord[2] + other['z'])

    def __sub__(self, other):
        if isinstance(other, Point):
            return Point(self._coord[0] - other['x'], self._coord[1] - other['y'], self._coord[2] - other['z'])

    def __pow__(self, power, modulo=None):
        return self._coord[0] ** power + self._coord[1] ** power + self._coord[2] ** power

    def __repr__(self):
        return '(%s, %s, %s)' % (self._coord[0], self._coord[1], self._coord[2],)


class Line():
    def __init__(self, a, b, c, x0, y0, z0, point1, point2):
        self._a = float(a)
        self._b = float(b)
        self._c = float(c)
        self._x0 = float(x0)
        self._y0 = float(y0)
        self._z0 = float(z0)
        self._p1 = point1
        self._p2 = point2

    def get_extrapolated_point(self, constraint, constraint_dim):
        if constraint_dim == 'x':
            val = (constraint - self._x0) / self._a

            x = constraint
            y = val * self._b + self._y0
            z = val * self._c + self._z0
        elif constraint_dim == 'y':
            val = (constraint - self._y0) / self._b

            x = val * self._a + self._x0
            y = constraint
            z = val * self._c + self._z0
        elif constraint_dim == 'z':
            val = (constraint - self._z0) / self._c

            x = val * self._a + self._x0
            y = val * self._b + self._y0
            z = constraint
        else:
            raise ValueError('Invalid constraint dim specified: %s, needs to be [x,y, or z]\r\n' % constraint_dim)

        return Point(x, y, z)

    @staticmethod
    def line_from_points(point1, point2):
        x0 = point1['x']
        y0 = point1['y']
        z0 = point1['z']

        a = point2['x'] - x0
        b = point2['y'] - y0
        c = point2['z'] - z0

        return Line(a, b, c, x0, y0, z0, point1, point2)

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

    def plot(self):
        x1 = self._a + self._x0
        y1 = self._b + self._y0

        plt.plot([self._x0, x1], [self._y0, y1])


# ----------------------------------------------------------------------------------------------------------------------
class Plane():
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
class Spline():
    def __init__(self, points, closed_loop=True, logger=None):
        if logger is None:
            logger = logging.getLogger()
        self.logger = logger

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

        self._calculate_coefficents()

    def _calculate_coefficents(self):
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

        while t < self.segments:
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

    def get_x_y_z(self, resolution=10, fixed_distance=None):
        """

        :param resolution:
        :return:
        """
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

    def get_points(self, resolution=10):
        x, y, z = self.get_x_y_z(resolution=resolution)

        points = list()
        for i in range(0, len(x)):
            points.append(Point(x[i], y[i], z[i]))

        return points

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


class GeometricFunctions():

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
    def path_length(path):
        length = 0
        if isinstance(path, list):
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
                raise ValueError('Error: Key Point 1: %s was never found on the path with start point: %s' %
                                 (point_1, path[0]))
        else:
            raise NotImplementedError('Error: path_length is currently only implemented for a list of points defining'
                                      ' the path')
        return length

    @staticmethod
    def get_point_along_path(path, distance, start_point=None):
        length = 0
        index = 0

        valid_region = True if start_point is None else False

        if isinstance(path, list):
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
    def _transform_dim(item, dim):
        if not isinstance(item, Point):
            if dim == 'x':
                dim = 0
            elif dim == 'y':
                dim = 1
            elif dim == 'z':
                dim = 2
        return dim
