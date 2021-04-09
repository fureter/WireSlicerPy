import os
import csv

import numpy as np
import matplotlib.pyplot as plt


# ----------------------------------------------------------------------------------------------------------------------
class Point(object):
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
        return self._coord[0] ** 2 + self._coord[1] ** 2 + self._coord[2] ** 2

    def __repr__(self):
        return '(%s, %s, %s)' % (self._coord[0], self._coord[1], self._coord[2],)


class Line(object):
    def __init__(self, a, b, c, x0, y0, z0):
        self._a = float(a)
        self._b = float(b)
        self._c = float(c)
        self._x0 = float(x0)
        self._y0 = float(y0)
        self._z0 = float(z0)

    def get_extrapolated_point(self, constraint, constraint_dim):
        if constraint_dim is 'x':
            val = (constraint - self._x0) / self._a

            x = constraint
            y = val * self._b + self._y0
            z = val * self._c + self._z0
        elif constraint_dim is 'y':
            val = (constraint - self._y0) / self._b

            x = val * self._a + self._x0
            y = constraint
            z = val * self._c + self._z0
        elif constraint_dim is 'z':
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

        return Line(a, b, c, x0, y0, z0)


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


# ----------------------------------------------------------------------------------------------------------------------
class Spline(object):
    def __init__(self, points, closed_loop=True):

        self._points = points
        if closed_loop:
            self._points = np.append(self._points, points[0])
            print(self._points)
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
        # loop through the 3 dimensions to get coefficents for x, y, and z
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

            # if self._loop:
            #     mat[2,4*self.segments-4] = (num_points) ** 3
            #     mat[2,4*self.segments-3] = (num_points) ** 2
            #     mat[2,4*self.segments-2] = (num_points) ** 1
            #     mat[2,4*self.segments-1] = 1
            #
            #     mat[3, 4 * self.segments - 4] = (1) ** 3
            #     mat[3, 4 * self.segments - 3] = (11) ** 2
            #     mat[3, 4 * self.segments - 2] = (1) ** 1
            #     mat[3, 4 * self.segments - 1] = 1
            #
            #     mat[4 * self.segments - 4, 4 * self.segments - 4] = 3*(num_points) ** 2
            #     mat[4 * self.segments - 4, 4 * self.segments - 3] = 2*(num_points) ** 1
            #     mat[4 * self.segments - 4, 4 * self.segments - 2] = 1
            #
            #     mat[4 * self.segments - 4, 0] = 3 * (num_points) ** 2
            #     mat[4 * self.segments - 4, 1] = 2 * (num_points) ** 1
            #     mat[4 * self.segments - 4, 2] = 1
            # else:
            mat[2, 0] = 6
            mat[2, 1] = 2

            mat[3, 4 * self.segments - 3] = 6 * num_points
            mat[3, 4 * self.segments - 2] = 2

            print('Equations: %s, Intervals: %s, Needed Eqns: %s\r\n' % (cnt, self.segments, (4 * self.segments)))
            print('Len Var: %s, Size Mat: %s\r\n' % (len(var), mat.shape))
            i_coeff = np.zeros(4 * self.segments)
            self._write_numerical_to_csv(mat=mat, i_coeff=i_coeff, var=var,
                                         directory=r'C:\Users\FurEt\PycharmProjects\WireSlicerPy',
                                         file_name=r'debug_spline_%s.csv' % ({0: 'x', 1: 'y', 2: 'z'}[i]))

            # print('mat: %s' % mat)
            # print('var: %s' % var)
            i_coeff = np.linalg.solve(mat, var)

            self._write_numerical_to_csv(mat=mat, i_coeff=i_coeff, var=var,
                                         directory=r'C:\Users\FurEt\PycharmProjects\WireSlicerPy',
                                         file_name=r'debug_spline_%s.csv' % ({0: 'x', 1: 'y', 2: 'z'}[i]))
            # print('coeff: %s' % i_coeff)

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

    def get_x_y_z(self, resolution=10):
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
            x[i] = self.ax[indx] * pos ** 3 + self.bx[indx] * pos ** 2 + self.cx[indx] * pos + self.dx[indx]
            y[i] = self.ay[indx] * pos ** 3 + self.by[indx] * pos ** 2 + self.cy[indx] * pos + self.dy[indx]
            z[i] = self.az[indx] * pos ** 3 + self.bz[indx] * pos ** 2 + self.cz[indx] * pos + self.dz[indx]

        return x, y, z

    def plot_spline(self):
        (x, y, z) = self.get_x_y_z()

        plt.plot(x, y)
        plt.draw()

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

        raise AttributeError('T(%s) out of range for the given intervals(%s' % (t, self.segments))
