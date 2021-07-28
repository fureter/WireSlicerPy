import copy
import logging
import sys
import timeit

import matplotlib.pyplot as plt
import numpy as np

import geometry.complex as comp
import geometry.primative as prim


class PointManip():
    class Transform():

        @staticmethod
        def translate(points, vector):
            if isinstance(vector, prim.Point):
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
                PointManip.Transform.translate(points, [-origin[0], -origin[1], -origin[2]])

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
    def reorder_2d_cw_subdivide(outer_points, inner_points):
        """
        Sorting method used specifically for subdivisions.
        :param outer_points:
        :param outer_points:
        :return:
        """
        pass

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
        center = prim.Point(center[0], center[1], center[2])

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
        else:
            raise NotImplementedError
        print('Took %ss to complete %s' % (timeit.default_timer() - start_polar, name))

        return np.array(sorted_points)

    @staticmethod
    def align_cross_sections_on_workpiece(cross_section_list, work_piece, output_dir, wire_cutter,
                                          distance_between_sections):
        """

        :param list[comp.CrossSectionPair] cross_section_list:
        :param prim.WorkPiece work_piece:
        :param WireCutter wire_cutter:
        :return:
        """
        logger = logging.getLogger(__name__)
        position_dict = dict()
        ind = 0
        for section in cross_section_list:
            position_dict[ind] = {'section': section, 'rot': 0, 'x_offset': 0, 'y_offset': 0,
                                  'collider': PointManip.create_section_collider(section, distance_between_sections)}
            ind += 1
        logger.info("Setting initial positions for Cross Sections on the work_piece")
        PointManip.set_initial_cross_section_offsets(position_dict, work_piece)
        prim.GeometricFunctions.plot_cross_sections_on_workpiece(position_dict, work_piece, output_dir, index='initial')
        plt.show()

        grid_resolution = 0.20
        x_grid = np.array(range(0, int(work_piece.width * grid_resolution + grid_resolution)),
                          dtype=np.float) / grid_resolution
        y_grid = np.array(range(0, int(work_piece.height * grid_resolution + grid_resolution)),
                          dtype=np.float) / grid_resolution
        logger.info("Creating the meshgrid to calculate the potential field over (dim [%s, %s])", x_grid.shape[0],
                    y_grid.shape[0])
        x_mesh, y_mesh = np.meshgrid(x_grid, y_grid)
        x_mesh = x_mesh.T
        y_mesh = y_mesh.T

        index = 0
        dt = 0.003
        iterations = 300
        while index < iterations:
            start = timeit.default_timer()
            potential_field = np.zeros([x_mesh.shape[0], x_mesh.shape[1], 2])
            logger.info("Calculating the potential field.")
            PointManip.calculate_potential_field(x_mesh, y_mesh, position_dict, 1.0, 25, potential_field,
                                                 work_piece=work_piece)
            logger.info("Plotting the potential field.")
            prim.GeometricFunctions.plot_potential_field(x_mesh, y_mesh, potential_field, output_dir, index)
            PointManip.move_position_dict_from_potential_field(position_dict, potential_field, dt, x_grid, y_grid,
                                                               work_piece)
            prim.GeometricFunctions.plot_cross_sections_on_workpiece(position_dict, work_piece, output_dir, index=index)

            index += 1
            logger.info('Finished iteration %s, took %ss', index, timeit.default_timer() - start)
            plt.close('all')

        logger.info("Performing the final translation of all CrossSectionPairs.")
        for ind in position_dict:
            prim.GeometricFunctions.move_cross_section_from_position_dict(
                path_1=position_dict[ind]['section'].section1.get_path(),
                path_2=position_dict[ind]['section'].section2.get_path(),
                dict_entry=position_dict[ind])

        logger.info("Plotting the CrossSectionPairs on the work_piece for visualization.")
        prim.GeometricFunctions.plot_cross_sections_on_workpiece(position_dict, work_piece, output_dir, index='final')


    @staticmethod
    def move_position_dict_from_potential_field(position_dict, potential_field, dt, x_grid, y_grid, work_piece):
        """

        :param dict[int, dict] position_dict:
        :param np.ndarray potential_field:
        :param float dt:
        :param x_grid:
        :param y_grid:
        :param prim.WorkPiece work_piece:
        :return:
        """
        logger = logging.getLogger(__name__)
        for ind in position_dict:
            center = prim.Point(0, 0, 0)
            path = position_dict[ind]['collider'].get_path()
            x_force = 0
            y_force = 0
            torque = 0
            for point in path:
                center += point
            center /= len(path)

            for point in path:
                x = point['x']
                y = point['y']
                x_coord = np.argmin(np.abs(x_grid-x))
                y_coord = np.argmin(np.abs(y_grid-y))

                force = potential_field[x_coord, y_coord, :]
                x_force += force[0]
                y_force += force[1]
                lever_arm = point - center
                torque += lever_arm['x'] * force[1] + lever_arm['y'] * force[0]

            # If the segment is outside of the work_piece, add additional force to bring it back in.
            if position_dict[ind]['x_offset'] < 0:
                x_force += 20
            elif position_dict[ind]['x_offset'] > work_piece.width:
                x_force -= 20
            if position_dict[ind]['y_offset'] < 0:
                y_force += 20
            elif position_dict[ind]['y_offset'] > work_piece.height:
                y_force -= 20

            delta_x = 100*(x_force * dt**2)/2  # Mass is assumed 1/100
            delta_y = 100*(y_force * dt**2)/2  # Mass is assumed 1/100
            delta_rot = 0.2*(torque * dt**2)/2  # Moment of Inertia is assumed 4

            #logger.debug('delta_x : %s, delta_y: %s, delta_rot: %s', delta_x, delta_y, np.rad2deg(delta_rot))

            position_dict[ind]['collider'].translate([delta_x, delta_y, 0])
            position_dict[ind]['x_offset'] += delta_x
            position_dict[ind]['y_offset'] += delta_y

            PointManip.Transform.rotate(path, [0, 0, delta_rot],
                                        origin=[position_dict[ind]['x_offset'], position_dict[ind]['y_offset'], 0])

            position_dict[ind]['rot'] += np.rad2deg(delta_rot)

    @staticmethod
    def calculate_potential_field(x_mesh, y_mesh, position_dict, alpha, beta, field, work_piece):
        logger = logging.getLogger(__name__)
        for ind_x in range(0, x_mesh.shape[0]):
            start = timeit.default_timer()
            for ind_y in range(0, y_mesh.shape[1]):
                x = x_mesh[ind_x, ind_y]
                y = y_mesh[ind_x, ind_y]

                closest_sections = PointManip.get_n_closest_sections(x=x, y=y, position_dict=position_dict, n=3)
                for section in closest_sections:
                    field[ind_x, ind_y, :] += alpha * PointManip.calculate_force_for_potential_field(x, y, section,
                                                                                                     work_piece=work_piece,
                                                                                                     beta=beta)
            # logger.info("Finished calculating the %s row for potential fields. Took: %ss", (ind_x + 1),
            #             timeit.default_timer() - start)

    @staticmethod
    def calculate_force_for_potential_field(x, y, section, work_piece, beta):
        """

        :param x:
        :param y:
        :param comp.CrossSection section:
        :return:
        """
        force = np.zeros(2)
        path = section.get_path()
        curr_point = prim.Point(x, y, 0)
        closest_point = path[0]
        closest_dist = np.sqrt((prim.Point(closest_point['x'], closest_point['y'], 0) - curr_point)**2)
        for point in section.get_path():
            dist = np.sqrt((prim.Point(point['x'], point['y'], 0) - curr_point)**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_point = point

        resultant_vector = closest_point - curr_point
        dist = np.sqrt(resultant_vector**2)
        if section.point_in_section(prim.Point(x, y, 0)):
            scale = abs((2*dist)**1.5)/10.0 * beta  # forcing value to get points outside of other collider box
            force[0] = scale*resultant_vector[0]
            force[1] = scale*resultant_vector[1]
        else:
            scale = np.log10(abs(dist/10.0)**1.5)/10.0 * beta
            force[0] = scale*resultant_vector[0]
            force[1] = scale*resultant_vector[1]

        # Add constant external force to move cross sections towards the upper left
        force[0] += -8*(x/100)
        force[1] += 4*((work_piece.height - y)/100)

        PointManip.add_wall_force(force, x=x, y=y, work_piece=work_piece, beta=6, wall_dist=25.0)

        return force

    @staticmethod
    def add_wall_force(force, x, y, work_piece, beta, wall_dist):
        if abs(x - work_piece.width) < wall_dist:
            force[0] += -max([15 * beta/abs(x - work_piece.width), wall_dist])
        elif abs(x) < wall_dist:
            force[0] += max([15 * beta/abs(x - work_piece.width), wall_dist])

        if abs(y - work_piece.height) < wall_dist:
            force[1] += -max([5.0 * beta/abs(y - work_piece.height), wall_dist])
        elif abs(y) < wall_dist:
            force[1] += max([5.0 * beta/abs(y - work_piece.height), wall_dist])


    @staticmethod
    def get_n_closest_sections(x, y, position_dict, n):
        section_list = list()
        dist_list = list()
        for key in position_dict:
            dist = np.sqrt((x-position_dict[key]['x_offset'])**2 + (y-position_dict[key]['y_offset'])**2)
            if len(section_list) < n:
                section_list.append(position_dict[key]['collider'])
                dist_list.append(dist)
            else:
                max_dist = max(dist_list)
                if max_dist > dist:
                    max_index = dist_list.index(max_dist)
                    dist_list[max_index] = dist
                    section_list[max_index] = position_dict[key]['collider']

        return section_list

    @staticmethod
    def create_section_collider(section, wall_thickness):
        """
        Creates an offset curve and compresses the curve to a lower point count to make collision detection more
        efficient. Uses the section with the larger perimeter to create the collider.

        :param comp.CrossSectionPair section:
        :return: Compressed path used for collision detection.
        :rtype: comp.CrossSection
        """
        larger_section = section.section1 if prim.GeometricFunctions.path_length(section.section1.get_path()) > \
                                             prim.GeometricFunctions.path_length(
                                                 section.section2.get_path()) else section.section2
        path = prim.GeometricFunctions.offset_curve(larger_section.get_path(), offset_scale=wall_thickness, dir=0,
                                                    divisions=2)
        return comp.CrossSection(prim.Path(prim.GeometricFunctions.normalize_path_points(path, num_points=32)))

    @staticmethod
    def set_initial_cross_section_offsets(position_dict, work_piece):
        """

        :param position_dict:
        :param prim.WorkPiece work_piece:
        :return:
        """
        max_x = work_piece.width
        max_y = work_piece.height
        bb = prim.GeometricFunctions.get_bounding_box_from_path(position_dict[0]['collider'].get_path())
        curr_x = (bb[1]['x'] - bb[0]['x'])/2.0
        curr_y = max_y - (bb[1]['y'] - bb[0]['y'])/2.0

        position_dict[0]['x_offset'] = curr_x
        position_dict[0]['y_offset'] = curr_y
        prim.GeometricFunctions.center_path(position_dict[0]['collider'].get_path())
        position_dict[0]['collider'].translate([curr_x, curr_y, 0])

        for ind in range(1, len(position_dict.keys())):
            prim.GeometricFunctions.center_path(position_dict[ind]['collider'].get_path())
            bound_box_curr = prim.GeometricFunctions.get_bounding_box_from_path(
                position_dict[ind]['collider'].get_path())

            if bound_box_curr[1][0] - bound_box_curr[0][0] < bound_box_curr[1][1] - bound_box_curr[0][1]:
                position_dict[ind]['rot'] = 90.0
                PointManip.Transform.rotate(position_dict[ind]['collider'].get_path(),
                                            [0, 0, np.deg2rad(position_dict[ind]['rot'])])

                bound_box_curr = prim.GeometricFunctions.get_bounding_box_from_path(
                    position_dict[ind]['collider'].get_path())

            bound_box_last = prim.GeometricFunctions.get_bounding_box_from_path(
                position_dict[ind - 1]['collider'].get_path())

            curr_x += (bound_box_last[1]['x'] - bound_box_last[0]['x'])/2.0 + \
                      (bound_box_curr[1]['x'] - bound_box_curr[0]['x'])/2.0

            if curr_x + (bound_box_curr[1]['x'] - bound_box_curr[0]['x'])/2 >= work_piece.width:
                curr_y -= (bound_box_last[1]['y'] - bound_box_last[0]['y'])/2 + \
                          (bound_box_curr[1]['y'] - bound_box_curr[0]['y'])/2
                curr_x = (bound_box_curr[1]['x'] - bound_box_curr[0]['x'])/2.0

            position_dict[ind]['x_offset'] = curr_x
            position_dict[ind]['y_offset'] = curr_y

            position_dict[ind]['collider'].translate([curr_x, curr_y, 0])

    @staticmethod
    def create_section_links_for_cross_section_pairs(cross_section_list):
        """

        :param list[comp.CrossSectionPair] cross_section_list:
        :return:
        :rtype: list[list[list[prim.SectionLink]]]
        """
        section1 = cross_section_list[0].section1.get_path()
        section2 = cross_section_list[0].section2.get_path()

        section_link_list = list()

        link_1_y = prim.SectionLink(prim.Point(0, 0, section1[0]['z']),
                                    prim.Point(0, section1[0]['y'], section1[0]['z']),
                                    fast_cut=True)
        link_2_y = prim.SectionLink(prim.Point(0, 0, section2[0]['z']),
                                    prim.Point(0, section2[0]['y'], section2[0]['z']),
                                    fast_cut=True)

        link_1_x = prim.SectionLink(prim.Point(section1[0]['x'], section1[0]['y'], section1[0]['z']),
                                    prim.Point(0, section1[0]['y'], section1[0]['z']),
                                    fast_cut=False)
        link_2_x = prim.SectionLink(prim.Point(section2[0]['x'], section2[0]['y'], section2[0]['z']),
                                    prim.Point(0, section2[0]['y'], section2[0]['z']),
                                    fast_cut=True)

        section_link_list.append([[link_1_y, link_2_y], [link_1_x, link_2_x]])

        for ind in range(1, len(cross_section_list)):
            prev_section_1 = cross_section_list[ind-1].section1.get_path()
            prev_section_2 = cross_section_list[ind-1].section2.get_path()
            next_section_1 = cross_section_list[ind].section1.get_path()
            next_section_2 = cross_section_list[ind].section2.get_path()

            link_1 = prim.SectionLink(prim.Point(prev_section_1[-1]['x'], prev_section_1[-1]['y'],
                                                 prev_section_1[-1]['z']),
                                      prim.Point(next_section_1[-1]['x'], next_section_1[-1]['y'],
                                                 next_section_1[-1]['z']),
                                      False)

            link_2 = prim.SectionLink(prim.Point(prev_section_2[-1]['x'], prev_section_2[-1]['y'],
                                                 prev_section_2[-1]['z']),
                                      prim.Point(next_section_2[-1]['x'], next_section_2[-1]['y'],
                                                 next_section_2[-1]['z']),
                                      False)

            section_link_list.append([[link_1, link_2]])
        return section_link_list

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
        step = -tol / 2
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
            dist = np.sqrt((curr_point - point) ** 2)
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
                    angle = np.rad2deg(np.arctan2(point['y'] - curr_point['y'], point['x'] - curr_point['x']))
                    angle_dict[angle] = point

            reference_line = PointManip.get_refernce_line(mid_lines, curr_point['x'], 'x')

            next_angle_cw = PointManip._get_next_angle_from_sorted_angles(angle_dict=angle_dict, curr_point=curr_point,
                                                                          check_angle=check_angle, tol=tol, step=step,
                                                                          reference_line=reference_line)
            next_angle_ccw = PointManip._get_next_angle_from_sorted_angles(angle_dict=angle_dict, curr_point=curr_point,
                                                                           check_angle=-check_angle, tol=tol,
                                                                           step=-step, reference_line=reference_line)
            if next_angle_ccw is not None and next_angle_cw is not None:
                dist_cw = 0.9 * np.sqrt((angle_dict[next_angle_cw] - curr_point) ** 2)
                dist_ccw = np.sqrt((angle_dict[next_angle_ccw] - curr_point) ** 2)

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

        line = prim.Line.line_from_points(left_most_point, prim.Point(closest_points[0][0]['x'],
                                                                      (closest_points[0][0]['y'] +
                                                                       closest_points[0][1]['y']) / 2,
                                                                      closest_points[0][0]['z']))
        # line.plot()
        lines.append(line)
        for indx in range(0, segments - 2):
            left = prim.Point(closest_points[indx][0]['x'],
                              (closest_points[indx][0]['y'] +
                               closest_points[indx][1]['y']) / 2,
                              closest_points[indx][0]['z'])

            right = prim.Point(closest_points[indx + 1][0]['x'],
                               (closest_points[indx + 1][0]['y'] +
                                closest_points[indx + 1][1]['y']) / 2,
                               closest_points[indx + 1][0]['z'])
            line = prim.Line.line_from_points(left, right)
            # line.plot()
            lines.append(line)

        last = prim.Point(closest_points[-1][0]['x'],
                          (closest_points[-1][0]['y'] +
                           closest_points[-1][1]['y']) / 2,
                          closest_points[-1][0]['z'])
        line = prim.Line.line_from_points(last, right_most_point)
        # line.plot()
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
            if abs(coord - point[dim]) < abs(coord - closest_points[0][dim]) and point != closest_points[1]:
                closest_points[1] = closest_points[0]
                closest_points[0] = point
                continue
            if abs(coord - point[dim]) < abs(coord - closest_points[1][dim]) and point != closest_points[0]:
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
                    dist = np.sqrt((angle_dict[key] - curr_point) ** 2)
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
