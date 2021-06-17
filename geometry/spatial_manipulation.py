import copy
import logging
import sys
import timeit

import numpy as np
import matplotlib.pyplot as plt

from .primative import Point
from .primative import Line


class PointManip():
    class Transform():

        @staticmethod
        def translate(points, vector):
            if isinstance(vector, Point):
                vector = [vector['x'], vector['y'], vector['z']]
            for point in points:
                point['x'] += vector[0]
                point['y'] += vector[1]
                point['z'] += vector[2]

        @staticmethod
        def rotate(points, vector, origin=None):
            cr = np.cos(vector[0])
            sr = np.sin(vector[0])
            cp = np.cos(vector[1])
            sp = np.sin(vector[1])
            cy = np.cos(vector[2])
            sy = np.sin(vector[2])
            dcm = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                            [-sp, cp * sr, cp * cr]])

            if origin is not None:
                PointManip.Transform.translate(points, [-origin[0],-origin[1],-origin[2]])

            for point in points:
                tmp = np.array([point['x'], point['y'], point['z']])
                new_point = dcm.dot(tmp)
                point['x'] = new_point[0]
                point['y'] = new_point[1]
                point['z'] = new_point[2]

            if origin is not None:
                PointManip.Transform.translate(points, origin)

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
                point['x'] = res[0]
                point['y'] = res[1]
                point['z'] = res[2]

    @staticmethod
    def reorder_2d_cw(points, method=0):
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

        start_polar = timeit.default_timer()
        if method == 0:
            sorted_points = PointManip._complex_polar_sort(points)
            name = 'complex_polar_sort'
        elif method == 1:
            sorted_points = PointManip._polar_sort(points, center)
            name = 'polar_sort'
        elif method == 2:
            sorted_points = PointManip._loop_dist_sort(points)
            name = 'loop_dist_sort'
        print('Took %ss to complete %s' % (timeit.default_timer() - start_polar, name))

        return np.array(sorted_points)

    @staticmethod
    def _polar_sort(points, center):
        logger = logging.getLogger(__name__)
        polar_coords = dict()
        keys = list()
        sorted_points = list()
        for p in points:
            theta = np.rad2deg(np.arctan2((p['y'] - center['y']), (p['x'] - center['x'])))
            theta -= 180
            if theta < 0:
                theta += 360

            keys.append(theta)
            polar_coords[theta] = p
        logger.debug('keys: %s', keys)
        rev_keys = copy.deepcopy(keys)
        rev_keys.sort(reverse=True)
        rev_keys_list = list()
        for key in rev_keys:
            rev_keys_list.append(key)
            sorted_points.append(polar_coords[key])

        logger.debug('Sorted keys: %s', rev_keys)

        return sorted_points

    @staticmethod
    def _complex_polar_sort(points):
        sorted_points = list()
        mid_lines = PointManip.get_mid_lines_from_closed_path(segments=15, points=points)
        tol = 10.0
        step = -tol/2
        check_angle = 90
        left_most_x = 10000000
        right_most_x = 0
        top_most_y = -10000000
        left_most_point = None
        for p in points:
            if p['x'] > right_most_x:
                right_most_x = p['x']
            if p['x'] < left_most_x:
                left_most_x = p['x']
                left_most_point = p
            if p['y'] > top_most_y:
                top_most_y = p['y']
        sorted_points.append(left_most_point)

        # Add the next point to the list using a nearest distance method, enforcing that the point must be above
        # the leading edge point. This is a weak condition to enforce CW looping.
        min_dist = sys.maxsize
        curr_point = sorted_points[-1]
        next_point = None
        for point in points:
            dist = np.sqrt((curr_point-point)**2)
            if dist < min_dist and point['y'] > curr_point['y']:
                next_point = point
                min_dist = dist
        sorted_points.append(next_point)

        # Start the primary sorting loop. Sorting will handle the top segment of the curve first by utilizing reference
        # lines that follow the midpoint of the cross section
        for indx in range(2, len(points)):
            curr_point = sorted_points[-1]
            angle_dict = dict()
            for point in points:
                if point not in sorted_points:
                    angle = np.rad2deg(np.arctan2(point['y'] - curr_point['y'], point['x']-curr_point['x']))
                    angle_dict[angle] = point

            reference_line = PointManip.get_refernce_line(mid_lines, curr_point['x'], 'x')

            next_angle_cw = PointManip._get_next_angle_from_sorted_angles(angle_dict=angle_dict, curr_point=curr_point,
                                                                          check_angle=check_angle, tol=tol, step=step,
                                                                          reference_line=reference_line)
            next_angle_ccw = PointManip._get_next_angle_from_sorted_angles(angle_dict=angle_dict, curr_point=curr_point,
                                                                           check_angle=-check_angle, tol=tol,
                                                                           step=-step, reference_line=reference_line)
            if next_angle_ccw is not None and next_angle_cw is not None:
                dist_cw = 0.9*np.sqrt((angle_dict[next_angle_cw] - curr_point)**2)
                dist_ccw = np.sqrt((angle_dict[next_angle_ccw] - curr_point)**2)

                next_angle = next_angle_cw if dist_cw < dist_ccw else next_angle_ccw
                sorted_points.append(angle_dict[next_angle])
        return sorted_points

    @staticmethod
    def get_mid_lines_from_closed_path(segments, points):

        lines = list()
        closest_points = list()
        left_most_x = sys.maxsize
        right_most_x = -sys.maxsize
        left_most_point = None
        right_most_point = None
        for p in points:
            if p['x'] > right_most_x:
                right_most_x = p['x']
                right_most_point = p
            if p['x'] < left_most_x:
                left_most_x = p['x']
                left_most_point = p
        chord = right_most_x - left_most_x
        delta = chord / segments
        for i in range(1, segments):
            x = left_most_x + i * delta
            closest_points.append(PointManip.get_two_points_at_coord(points, 'x', x))

        line = Line.line_from_points(left_most_point, Point(closest_points[0][0]['x'],
                                                            (closest_points[0][0]['y'] +
                                                             closest_points[0][1]['y'])/2,
                                                            closest_points[0][0]['z']))
        #line.plot()
        lines.append(line)
        for indx in range(0, segments-2):
            left = Point(closest_points[indx][0]['x'],
                         (closest_points[indx][0]['y'] +
                          closest_points[indx][1]['y']) / 2,
                         closest_points[indx][0]['z'])

            right = Point(closest_points[indx + 1][0]['x'],
                          (closest_points[indx + 1][0]['y'] +
                           closest_points[indx + 1][1]['y']) / 2,
                          closest_points[indx + 1][0]['z'])
            line = Line.line_from_points(left, right)
            # line.plot()
            lines.append(line)

        last = Point(closest_points[-1][0]['x'],
                     (closest_points[-1][0]['y'] +
                      closest_points[-1][1]['y']) / 2,
                     closest_points[-1][0]['z'])
        line = Line.line_from_points(last, right_most_point)
        #line.plot()
        lines.append(line)

        return lines

    @staticmethod
    def get_two_points_at_coord(points, dim, coord):
        """

        :param list[Point] points:
        :param str or int dim:
        :param float or int coord:
        :return:
        """
        closest_points = [None, None]
        for point in points:
            if closest_points[0] is None:
                closest_points[0] = point
                continue
            if closest_points[1] is None:
                closest_points[1] = point
                continue
            if abs(coord-point[dim]) < abs(coord-closest_points[0][dim]) and point != closest_points[1]:
                closest_points[1] = closest_points[0]
                closest_points[0] = point
                continue
            if abs(coord-point[dim]) < abs(coord-closest_points[1][dim]) and point != closest_points[0]:
                closest_points[1] = point
                continue

        return closest_points

    @staticmethod
    def get_refernce_line(lines, coord, dim):
        ret_val = None
        for line in lines:
            if line.coord_in_range_dim(coord, dim):
                ret_val = line
        return ret_val

    @staticmethod
    def _get_next_angle_from_sorted_angles(angle_dict, curr_point, tol=10, check_angle=90, step=1, reference_line=None):
        """
        
        :param dict[Point] angle_dict: 
        :param Point curr_point: 
        :param float or int tol:
        :param float or int check_angle: 
        :param float or int step: 
        :param Line or None reference_line:
        :return:
        """
        keys = angle_dict.keys()
        break_out = check_angle + 360 if step > 0 else check_angle - 360
        ret_val = None
        no_angle = True

        side_curr = np.sign(reference_line.signed_distance_to_point_xy(curr_point))

        while no_angle:
            within_tol = list()
            for key in keys:
                angle_diff = (check_angle - key)
                angle_diff = (angle_diff + 180) % 360 - 180
                signed_key = np.sign(reference_line.signed_distance_to_point_xy(angle_dict[key]))
                if abs(angle_diff) < tol and (signed_key == side_curr or side_curr == 0 or signed_key == 0):
                    within_tol.append(key)
            check_angle += step
            if check_angle == break_out:
                no_angle = False
            if len(within_tol) > 0:
                no_angle = False
                min_dist = sys.maxsize
                for key in within_tol:
                    dist = np.sqrt((angle_dict[key] - curr_point)**2)
                    if dist < min_dist:
                        min_dist = dist
                        ret_val = key

        return ret_val

    @staticmethod
    def _loop_dist_sort(points):
        sorted_points = list()
        left_most_x = sys.maxsize
        right_most_x = -sys.maxsize
        left_most_point = None
        for p in points:
            if p['x'] > right_most_x:
                right_most_x = p['x']
            if p['x'] < left_most_x:
                left_most_x = p['x']
                left_most_point = p

        sorted_points.append(left_most_point)
        for i in range(0, len(points)):
            curr_point = sorted_points[-1]
            closest_point = None
            min_dist = 1000000000
            for p in points:
                if p not in sorted_points:
                    # Use a weighted distance that places more emphasis on the x_distance, this helps with the trailing
                    # edge, highly cambered profiles will still be incorrect
                    dist = np.sqrt((curr_point['x'] - p['x']) ** 2 + (curr_point['y'] - p['y']) ** 2)
                    if dist < min_dist and p not in sorted_points:
                        min_dist = dist
                        closest_point = p
                        # take care of the trailing edge with overlapping x coords
            if closest_point is not None:
                sorted_points.append(closest_point)

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
                return None

            check_point['x'] = check_point['x'] + step * np.cos(direction)
            check_point['y'] = check_point['y'] + step * np.sin(direction)

            for p in points:
                dist = np.sqrt(np.sum((p - check_point) ** 2))

                if dist < min_dist:
                    min_dist = dist
                if dist < tol:
                    if debug:
                        x = np.array([origin['x'], check_point['x']])
                        y = np.array([origin['y'], check_point['y']])
                        plt.plot(x, y)
                        plt.draw()
                    return p
            num_step += 1
