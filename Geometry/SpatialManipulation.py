import copy
import timeit

import numpy as np
import matplotlib.pyplot as plt

from .PrimativeGeometry import Point


class PointManip(object):
    class Transform(object):

        @staticmethod
        def translate(points, vector):
            print('-' * 80)
            for point in points:
                print('before translate: %s' % point)
                point['x'] += vector[0]
                point['y'] += vector[1]
                point['z'] += vector[2]
                print('After translate: %s' % point)
            print('-' * 80)

        @staticmethod
        def rotate(points, vector):
            cr = np.cos(vector[0])
            sr = np.sin(vector[0])
            cp = np.cos(vector[1])
            sp = np.sin(vector[1])
            cy = np.cos(vector[2])
            sy = np.sin(vector[2])
            dcm = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                            [-sp, cp * sr, cp * cr]])

            for point in points:
                tmp = np.array([point['x'], point['y'], point['z']])
                new_point = dcm.dot(tmp)
                point['x'] = new_point[0]
                point['y'] = new_point[1]
                point['z'] = new_point[2]

        @staticmethod
        def scale(points, scale, origin=None):
            scale_mat = np.array([[scale[0], 0, 0, 0],
                                  [0, scale[1], 0, 0],
                                  [0, 0, scale[2], 0],
                                  [0, 0, 0, 1]])
            transform_mat = scale_mat

            if origin is not None:
                translation_mat = np.array([[1, 0, 0, -origin[0]],
                                            [0, 1, 0, -origin[1]],
                                            [0, 0, 1, -origin[2]],
                                            [0, 0, 0, 1]])
                inv_translation_mat = np.linalg.inv(translation_mat)
                tmp = np.dot(inv_translation_mat, scale_mat)
                transform_mat = np.dot(tmp, translation_mat)

            for point in points:
                tmp = np.array([point['x'], point['y'], point['z'], 1])
                res = transform_mat.dot(tmp)
                print('point: %s' % point)
                print('res: %s' % res)
                point['x'] = res[0]
                point['y'] = res[1]
                point['z'] = res[2]

    @staticmethod
    def reorder_2d_cw(points):
        """
        takes a list of points and reorders the list so that the list form a continuous line rotating clockwise
        assumes x and y are the two dimensions of interest
        :return: reordered list of points
        """
        center = [0, 0, 0]
        for i in range(0, len(points)):
            center[0] += points[i]['x']
            center[1] += points[i]['y']
            center[2] += points[i]['z']

        center[0] = center[0] / len(points)
        center[1] = center[1] / len(points)
        center[2] = center[2] / len(points)
        center = Point(center[0], center[1], center[2])
        print('Center: %s' % center)

        start_polar = timeit.default_timer()
        sorted_points = PointManip._loop_dist_sort(points)
        stop_polar = timeit.default_timer()

        print('polar time: %f' % (stop_polar - start_polar))

        return np.array(sorted_points)

    @staticmethod
    def _polar_sort(points, center):
        print(center)
        polar_coords = dict()
        keys = list()
        sorted_points = list()
        for p in points:
            theta = np.arctan2((p['y'] - center['y']), (p['x'] - center['x']))
            keys.append(theta)
            polar_coords[theta] = p
        keys.sort()
        for key in keys:
            sorted_points.append(polar_coords[key])

        return sorted_points

    @staticmethod
    def _loop_dist_sort(points):
        sorted_points = list()
        left_most_x = 10000000
        right_most_x = 0
        left_most_point = None
        for p in points:
            if p['x'] > right_most_x:
                right_most_x = p['x']
            if p['x'] < left_most_x:
                left_most_x = p['x']
                left_most_point = p

        sorted_points.append(left_most_point)
        trailing_edge_um = True
        for i in range(1, len(points)):
            curr_point = sorted_points[-1]
            closest_point = None
            min_dist = 1000000000
            y_weight = 4
            for p in points:
                # dist = np.sqrt((curr_point - p)**2)
                dist = np.sqrt((curr_point['x'] - p['x']) ** 2 + (y_weight * (curr_point['y'] - p['y'])) ** 2)
                if dist < min_dist and p not in sorted_points:
                    # print('dist: %s' % dist)
                    min_dist = dist
                    closest_point = p
                    # take care of the trailing edge with overlapping x coords
            if closest_point is not None:
                if closest_point['x'] == right_most_x and trailing_edge_um:
                    for pon in points:
                        if pon is not closest_point and pon['x'] == right_most_x:
                            c_l = closest_point - sorted_points[-1]
                            ang_c_l = np.rad2deg(np.arctan2(c_l['y'], c_l['x']))
                            p_l = sorted_points[-1] - sorted_points[-2]
                            ang_p_l = np.rad2deg(np.arctan2(p_l['y'], p_l['x']))
                            pon_l = pon - sorted_points[-1]
                            ang_pon_l = np.rad2deg(np.arctan2(pon_l['y'], pon_l['x']))

                            dist_curr = abs(ang_c_l - ang_p_l)
                            dist_alt = abs(ang_pon_l - ang_p_l)

                            print('ang last: %s\nang curr: %s\rang alt:%s\r' % (ang_p_l, ang_c_l, ang_pon_l))
                            print('dist_curr: %s, dist_alt: %s' % (dist_curr, dist_alt))

                            if dist_alt < dist_curr:
                                closest_point = pon
                            trailing_edge_um = False

                # print('Adding p:%s after p0:%s with a dist of: %s' % (closest_point, sorted_points[-1], min_dist))
                sorted_points.append(closest_point)

        # print('points: %s' % sorted_points)
        return sorted_points

    @staticmethod
    def _raycast(points, origin, direction, step, tol, max_step=258, debug=False):
        """
        returns the closest point to the ray travelling from point origin in the direction dir

        :param points:
        :param origin:
        :param direction:
        :return:
        """
        min_dist = 999999
        check_point = copy.deepcopy(origin)
        num_step = 0
        while True:
            if num_step > max_step:
                if debug:
                    # x = np.array([origin['x'], check_point['x']])
                    # y = np.array([origin['y'], check_point['y']])
                    # print('1x: %s, y: %s\r\n' % (x,y))
                    # plt.plot(x, y)
                    # plt.draw()
                    pass
                return None

            check_point['x'] = check_point['x'] + step * np.cos(direction)
            check_point['y'] = check_point['y'] + step * np.sin(direction)
            # print('Check Point: %s' % check_point)

            for p in points:
                dist = np.sqrt(np.sum((p - check_point) ** 2))

                if dist < min_dist:
                    min_dist = dist
                if dist < tol:
                    if debug:
                        x = np.array([origin['x'], check_point['x']])
                        y = np.array([origin['y'], check_point['y']])
                        # print('2x: %s, y: %s\r\n' % (x,y))
                        plt.plot(x, y)
                        plt.draw()
                    return p
            num_step += 1

            # print('err: %s, prev_err: %s'%(err, prev_err))
