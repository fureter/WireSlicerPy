import copy
import logging
import os
import sys
import timeit

import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
import numpy as np
import rtree

import geometry.complex as comp
import geometry.primative as prim
import geometry.spatial_manipulation as spma


class SpatialPlacement:
    MAX_VEL = 10000.0
    MIN_DT = 1e-3

    def __init__(self, work_piece, wire_cutter):
        self.logger = logging.getLogger(__name__)
        self.state_dict = dict()
        self.r_index = rtree.index.Index()
        self.work_piece = work_piece
        self.section_order = None
        self.unplaced_ind = list()
        self.wire_cutter = wire_cutter

    def bin_packing_algorithm(self, cross_section_list, output_dir, distance_between_sections):
        ind = 0
        for section in cross_section_list:
            prim.GeometricFunctions.center_path(section.get_path()[0])
            prim.GeometricFunctions.center_path(section.get_path()[1])
            collider = SpatialPlacement.create_section_collider(section, distance_between_sections)
            bb = prim.GeometricFunctions.get_bounding_box_from_path(collider.get_path())
            bb_center = prim.Point(x=(bb[1]['x'] + bb[0]['x']) / 2, y=(bb[1]['y'] + bb[0]['y']) / 2, z=0)
            path_center = prim.GeometricFunctions.get_center_of_path(collider.get_path())
            diff_centers = path_center - bb_center
            longest = np.max([abs(bb[1]['x'] - bb[0]['x']), abs(bb[1]['y'] - bb[0]['y'])])
            dist = np.sqrt(diff_centers ** 2)

            ang_from_centers = np.arctan2(diff_centers['y'], diff_centers['x'])
            ang_from_centers = ang_from_centers + 2 * np.pi if ang_from_centers < 0 else ang_from_centers

            if 0 < ang_from_centers < np.pi / 2:
                rot = 90
            elif np.pi / 2 < ang_from_centers < np.pi:
                rot = 0
            elif np.pi < ang_from_centers < 3 * np.pi / 2:
                rot = -90
            else:
                rot = 180
            # TODO: Attempting to orient the pieces so that they are open towards the right
            rot = -ang_from_centers

            config = 'L' if dist / longest > 0.07 else 'I'
            rot = rot #if config == 'L' else 0

            self.state_dict[ind] = {'section': section, 'rot': rot, 'x_pos': 0, 'y_pos': 0,
                                    'collider': collider, 'bb': bb, 'config': config}
            spma.PointManip.Transform.rotate(collider.get_path(), [0, 0, np.deg2rad(rot)], origin=path_center)
            prim.GeometricFunctions.plot_path(collider.get_path(), color=None)
            plt.plot([bb[0][0], bb[0][0], bb[1][0], bb[1][0], bb[0][0]],
                     [bb[0][1], bb[1][1], bb[1][1], bb[0][1], bb[0][1]], 'k')
            plt.scatter([bb_center['x']], [bb_center['y']], c='r')
            plt.scatter([path_center['x']], [path_center['y']], c='r')
            plt.quiver([bb_center['x']], [bb_center['y']], [diff_centers['x']], [diff_centers['y']])
            plt.axis('equal')
            plt.title('Rot Estimation: %s, ang: %s, ratio: %s, Config: %s' % (rot, np.rad2deg(ang_from_centers),
                                                                              dist / longest, config))
            plt.show()

            ind += 1

        def collider_size(entry):
            bb_i = self.state_dict[entry]['bb']
            return (bb_i[1][0] - bb_i[0][0]) * (bb_i[1][1] - bb_i[0][1])

        ordered_state_indexes = sorted(self.state_dict, key=collider_size)
        ordered_state_dict = dict()

        index = 0
        for ind in ordered_state_indexes:
            ordered_state_dict[index] = self.state_dict[ind]
            index += 1

        r_id = 0

        prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
                                                                 index='initial')

        last_pos_x = 0
        last_pos_y = self.work_piece.height
        indx = 0

        for ind in self.state_dict:
            item = self.state_dict[ind]
            config = item['config']
            if config == 'I' or config == 'L':
                ar = (item['bb'][1][1] - item['bb'][0][1]) / (item['bb'][1][0] - item['bb'][0][0])
                if ar < 1 and config != 'L':
                    item['rot'] = 90
                    spma.PointManip.Transform.rotate(item['collider'].get_path(), [0, 0, np.deg2rad(item['rot'])])
                placed = False
                while not placed:
                    delta_x = last_pos_x - item['x_pos']
                    delta_y = last_pos_y - item['y_pos']
                    spma.PointManip.Transform.translate(item['collider'].get_path(), [delta_x, delta_y, 0])
                    item['x_pos'] = last_pos_x
                    item['y_pos'] = last_pos_y
                    bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider'].get_path())

                    # Make sure the current bounding box is not outside of the work piece, we check against the
                    # top right corner first, then check quad tree intersections
                    if bb_curr[0]['x'] < 0:
                        delta_x = -bb_curr[0]['x']
                        spma.PointManip.Transform.translate(item['collider'].get_path(), [delta_x, 0, 0])
                        item['x_pos'] += delta_x
                        last_pos_x += delta_x
                        bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider'].get_path())
                    if bb_curr[1]['y'] > self.work_piece.height:
                        delta_y = self.work_piece.height - bb_curr[1]['y']
                        spma.PointManip.Transform.translate(item['collider'].get_path(), [0, delta_y, 0])
                        item['y_pos'] += delta_y
                        last_pos_y += delta_y
                        bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider'].get_path())
                    (left, bottom, right, top) = bb_curr[0]['x'], bb_curr[0]['y'], bb_curr[1]['x'], bb_curr[1]['y']
                    intersections = list(self.r_index.intersection((left, bottom, right, top)))
                    if len(intersections) > 0:
                        last_pos_x += 1
                    elif right > self.work_piece.width:
                        last_pos_x = 0
                        last_pos_y = 0
                    else:
                        if bb_curr[1]['x'] < self.work_piece.width:
                            y_set = False
                            while not y_set:
                                if bb_curr[1]['y'] < self.work_piece.height:
                                    delta_y = 1
                                else:
                                    delta_y = -1
                                    y_set = True
                                spma.PointManip.Transform.translate(item['collider'].get_path(), [0, delta_y, 0])
                                bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(
                                    item['collider'].get_path())
                                last_pos_y += delta_y
                                item['y_pos'] += delta_y

                                (left, bottom, right, top) = bb_curr[0]['x'], bb_curr[0]['y'], bb_curr[1]['x'], \
                                                             bb_curr[1]['y']
                                intersections = list(self.r_index.intersection((left, bottom, right, top)))
                                if len(intersections) > 0:
                                    delta_y = -1
                                    spma.PointManip.Transform.translate(item['collider'].get_path(), [0, delta_y, 0])
                                    bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(
                                        item['collider'].get_path())
                                    last_pos_y += delta_y
                                    item['y_pos'] += delta_y
                                    y_set = True

                            self.r_index.insert(id=r_id, coordinates=(left, bottom, right, top))
                            r_id += 1
                            placed = True
                        else:
                            delta_x = -left
                            spma.PointManip.Transform.translate(item['collider'].get_path(), [delta_x, 0, 0])
                            bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider'].get_path())
                            (left, bottom, right, top) = bb_curr[0]['x'], bb_curr[0]['y'], bb_curr[1]['x'], \
                                                         bb_curr[1]['y']
                            last_pos_y += delta_x
                            item['y_pos'] += delta_x
            prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
                                                                     index='%s' % indx)
            indx += 1
        prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
                                                                 index='final')
        self.apply_state_dict()

    def apply_state_dict(self):
        for entry in self.state_dict.values():
            center = self.wire_cutter.wire_length/2
            translate1 = [entry['x_pos'], entry['y_pos'], center - self.work_piece.thickness]
            translate2 = [entry['x_pos'], entry['y_pos'], center + self.work_piece.thickness]
            sec_1_path = entry['section'].section1.get_path()
            sec_2_path = entry['section'].section2.get_path()
            spma.PointManip.Transform.rotate(sec_1_path, [0, 0, np.deg2rad(entry['rot'])])
            spma.PointManip.Transform.rotate(sec_2_path, [0, 0, np.deg2rad(entry['rot'])])
            spma.PointManip.Transform.translate(sec_1_path, translate1)
            spma.PointManip.Transform.translate(sec_2_path, translate2)
            entry['bb'] = prim.GeometricFunctions.get_bounding_box_from_path(entry['collider'].get_path())
            assert len(sec_1_path) > 1, 'Error: Section 1 of State dict entry [%s] has no Points.' % entry
            assert len(sec_2_path) > 1, 'Error: Section 2 of State dict entry [%s] has no Points.' % entry

    def create_section_links_for_cross_section_pairs(self):
        """

        :return:
        :rtype: list[list[prim.SectionLink or comp.CrossSection]]
        """

        cut_list_1 = list()
        cut_list_2 = list()

        z_1 = self.state_dict[0]['section'].section1.get_path()[0]['z']
        z_2 = self.state_dict[0]['section'].section2.get_path()[0]['z']

        section_order_list = self._get_section_order()
        prev_point_1 = prim.Point(0, 0, z_1)
        prev_point_2 = prim.Point(0, 0, z_2)
        for y_axis in section_order_list:
            min_ind = prim.GeometricFunctions.get_index_min_coord(
                self.state_dict[y_axis[0]]['section'].section1.get_path(), 'x')
            y_start = self.state_dict[y_axis[0]]['section'].section1.get_path()[min_ind]['y']
            cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1), fast_cut=True))
            cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2), fast_cut=True))
            prev_point_1 = prim.Point(0, y_start, z_1)
            prev_point_2 = prim.Point(0, y_start, z_2)
            for x_axis in y_axis:
                section = self.state_dict[x_axis]['section']
                section_1 = section.section1.get_path()
                section_2 = section.section2.get_path()

                err_prompt = 'Error Sections do not have the same number of points S1 %s, S2 %s' % \
                             (len(section_1), len(section_2))
                assert abs(len(section_1) - len(section_2)) <= 1, err_prompt

                min_ind = prim.GeometricFunctions.get_index_min_coord(section_1, 'x')
                max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, 'x')

                cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(section_1[min_ind]['x'],
                                                                            section_1[min_ind]['y'], z_1)))
                cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(section_2[min_ind]['x'],
                                                                            section_2[min_ind]['y'], z_2)))

                prev_point_1, prev_point_2 = SpatialPlacement.add_section(cut_list_1, cut_list_2, False, section_1,
                                                                          section_2, z_1, z_2)
                if prev_point_1 is None:
                    prev_point_1 = prim.Point(section_1[max_ind]['x'],
                                              section_1[max_ind]['y'], z_1)
                    prev_point_2 = prim.Point(section_2[max_ind]['x'],
                                              section_2[max_ind]['y'], z_2)

            for x_axis in reversed(y_axis):
                section = self.state_dict[x_axis]['section']
                section_1 = section.section1.get_path()
                section_2 = section.section2.get_path()

                min_ind = prim.GeometricFunctions.get_index_min_coord(section_1, 'x')
                max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, 'x')

                if prev_point_1 != prim.Point(section_1[max_ind]['x'], section_1[max_ind]['y'], z_1):
                    cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(section_1[max_ind]['x'],
                                                                                section_1[max_ind]['y'], z_1)))
                    cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(section_2[max_ind]['x'],
                                                                                section_2[max_ind]['y'], z_2)))

                prev_point_1, prev_point_2 = SpatialPlacement.add_section(cut_list_1, cut_list_2, True, section_1,
                                                                          section_2, z_1, z_2)

            cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1)))
            cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2)))

            prev_point_1 = prim.Point(0, y_start, z_1)
            prev_point_2 = prim.Point(0, y_start, z_2)

        return cut_list_1, cut_list_2

    @staticmethod
    def add_section(cut_list_1, cut_list_2, reverse_dir, section_1, section_2, z_1, z_2):
        min_ind = prim.GeometricFunctions.get_index_min_coord(section_1, 'x')
        max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, 'x')
        point1 = None
        point2 = None
        if min_ind > max_ind:
            if reverse_dir:
                cut_list_1.append(comp.CrossSection(prim.Path(section_1[max_ind:min_ind+1])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[max_ind:min_ind+1])))
                point1 = section_1[min_ind]
                point2 = section_2[min_ind]
            else:
                cut_list_1.append(comp.CrossSection(prim.Path(section_1[min_ind:])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[min_ind:])))
                cut_list_1.append(comp.CrossSection(prim.Path(section_1[0:max_ind+1])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[0:max_ind+1])))
                point1 = section_1[max_ind]
                point2 = section_2[max_ind]

        else:
            if reverse_dir:
                cut_list_1.append(comp.CrossSection(prim.Path(section_1[max_ind:])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[max_ind:])))

                cut_list_1.append(comp.CrossSection(prim.Path(section_1[0:min_ind+1])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[0:min_ind+1])))
                point1 = section_1[min_ind]
                point2 = section_2[min_ind]

            else:
                cut_list_1.append(comp.CrossSection(prim.Path(section_1[min_ind:max_ind+1])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[min_ind:max_ind+1])))
                point1 = section_1[max_ind]
                point2 = section_2[max_ind]

        if point1:
            point1['z'] = z_1
            point2['z'] = z_2
        return point1, point2

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
                                                    divisions=1, add_leading_edge=False)

        path = prim.GeometricFunctions.normalize_path_points(path, num_points=32)
        path = prim.GeometricFunctions.close_path(path)

        return comp.CrossSection(prim.Path(path))

    def _get_section_order(self):
        """

        :return: List of lists containing the order in which to cut cross sections.
        :rtype: list[list[int]]
        """
        section_order_dict = dict()
        section_order_list = list()
        ind_taken = list()

        # Grab the entries that are along the y_axis boundary, these are used as the anchors to grab the remaining
        # pieces
        for ind in self.state_dict:
            if self.state_dict[ind]['bb'][0]['x'] <= 0:
                y_pos = (self.state_dict[ind]['bb'][1]['y'] + self.state_dict[ind]['bb'][0]['y']) / 2
                x_pos = (self.state_dict[ind]['bb'][1]['x'] + self.state_dict[ind]['bb'][0]['x']) / 2
                section_order_dict[y_pos] = dict()
                section_order_dict[y_pos][x_pos] = ind
                ind_taken.append(ind)

        for ind in self.state_dict:
            if ind not in ind_taken:
                for ind_y in section_order_dict:
                    if self.state_dict[ind]['bb'][0]['y'] <= ind_y < self.state_dict[ind]['bb'][1]['y']:
                        x_pos = (self.state_dict[ind]['bb'][1]['x'] + self.state_dict[ind]['bb'][0]['x']) / 2
                        section_order_dict[ind_y][x_pos] = ind
                        ind_taken.append(ind)
                        break
                if ind not in ind_taken:
                    self.unplaced_ind.append(ind)

        for ind in self.unplaced_ind:
            min_dist = 1000000
            closest_y = None
            for ind_y in section_order_dict:
                y_min = self.state_dict[ind]['bb'][0]['y']
                y_max = self.state_dict[ind]['bb'][1]['y']
                closest = min([abs(y_min - ind_y), abs(y_max - ind_y)])
                if closest < min_dist:
                    min_dist = closest
                    closest_y = ind_y

            x_pos = (self.state_dict[ind]['bb'][1]['x'] + self.state_dict[ind]['bb'][0]['x']) / 2
            section_order_dict[closest_y][x_pos] = ind

        for ind_y in sorted(section_order_dict, reverse=True):
            tmp_lst = list()
            for ind_x in sorted(section_order_dict[ind_y]):
                tmp_lst.append(section_order_dict[ind_y][ind_x])
            section_order_list.append(tmp_lst)

        self.section_order = section_order_list
        return self.section_order

    def plot_section_order(self, output_dir):
        plt.close('all')
        plt.figure(figsize=(16, 9), dpi=320)
        for ind1 in range(0, len(self.section_order)):
            for ind2 in range(0, len(self.section_order[ind1])):
                ind = self.section_order[ind1][ind2]
                prim.GeometricFunctions.plot_path(path=self.state_dict[ind]['section'].section1.get_path(),
                                                  color='C%s' % ind1, scatter=False)
                prim.GeometricFunctions.plot_path(path=self.state_dict[ind]['section'].section2.get_path(),
                                                  color='C%s' % ind1, scatter=False)
                txt = plt.text(self.state_dict[ind]['x_pos'], self.state_dict[ind]['y_pos'],
                               s='C%s, O(%s,%s)' % (ind, ind1, ind2), fontsize='x-small')
                txt.set_path_effects([pe.withStroke(linewidth=2, foreground='w')])

            plt.axis('equal')
            plt.savefig(os.path.join(output_dir, 'Cross_Section_cut_order.png'))

    def plot_section_splitting_debug(self, output_dir):
        section_order_list = self._get_section_order()
        ind = 0
        for y_axis in section_order_list:
            for x_axis in y_axis:
                plt.close('all')
                plt.figure(figsize=(16, 9), dpi=320)
                section = self.state_dict[x_axis]['section']
                section_1 = section.section1.get_path()
                section_2 = section.section2.get_path()

                min_ind = prim.GeometricFunctions.get_index_min_coord(section_1, 'x')
                max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, 'x')

                if min_ind < max_ind:
                    prim.GeometricFunctions.plot_path(section_1[min_ind:max_ind+1], color='C1', scatter=False)
                    prim.GeometricFunctions.plot_path(section_1[max_ind:], color='C2', scatter=False)
                    prim.GeometricFunctions.plot_path(section_1[0:min_ind+1], color='C2', scatter=False)

                    prim.GeometricFunctions.plot_path(section_2[min_ind:max_ind+1], color='C3', scatter=False)
                    prim.GeometricFunctions.plot_path(section_2[max_ind:], color='C4', scatter=False)
                    prim.GeometricFunctions.plot_path(section_2[0:min_ind+1], color='C4', scatter=False)
                else:
                    prim.GeometricFunctions.plot_path(section_1[min_ind:], color='C5', scatter=False)
                    prim.GeometricFunctions.plot_path(section_1[0:max_ind+1], color='C5', scatter=False)
                    prim.GeometricFunctions.plot_path(section_1[max_ind:min_ind+1], color='C6', scatter=False)

                    prim.GeometricFunctions.plot_path(section_2[min_ind:], color='C7', scatter=False)
                    prim.GeometricFunctions.plot_path(section_2[0:max_ind+1], color='C7', scatter=False)
                    prim.GeometricFunctions.plot_path(section_2[max_ind:min_ind+1], color='C8', scatter=False)
                plt.axis('equal')
                plt.savefig(os.path.join(output_dir, 'Cross_Section_cut_splitting_debug_%s.png' % ind))
                ind += 1


    class Depricated:
        @staticmethod
        def align_cross_sections_on_workpiece_single(cross_section_list, work_piece, output_dir, wire_cutter,
                                                     distance_between_sections):
            """

            :param list[comp.CrossSectionPair] cross_section_list:
            :param prim.WorkPiece work_piece:
            :param WireCutter wire_cutter:
            :return:
            """
            logger = logging.getLogger(__name__)
            state_dict = list()
            ind = 0
            for section in cross_section_list:
                state_dict.append({'section': section, 'rot': 0, 'omega': 0, 'x_pos': 0, 'y_pos': 0, 'v': 10, 'u': -15,
                                   'prev_rot': 0, 'prev_omega': 0, 'prev_x': 0, 'prev_y': 0, 'prev_v': 0, 'prev_u': 0,
                                   'collider': SpatialPlacement.create_section_collider(section, distance_between_sections),
                                   'collisions': [], 'prev_collisions': [], 'mass': 1, 'I': 1e3})
                if ind > 0:
                    state_dict[ind - 1]['mass'] = 1e12
                    state_dict[ind - 1]['I'] = 1e12
                    state_dict[ind - 1]['u'] = 0
                    state_dict[ind - 1]['v'] = 0
                    state_dict[ind - 1]['omega'] = 0

                logger.info("Setting initial positions for Cross Sections on the work_piece")
                SpatialPlacement.set_initial_cross_section_offsets(state_dict, work_piece)
                prim.GeometricFunctions.plot_cross_sections_on_workpiece(state_dict, work_piece, output_dir,
                                                                         index='initial_%s' % ind)
                plt.show()

                prev_used_space = SpatialPlacement.used_workpiece_area(state_dict)

                index = 0
                dt_0 = 0.02
                dt = dt_0
                iterations = 600
                max_force_lst = list()
                dt_lst = list()
                work_piece_copy = copy.deepcopy(work_piece)
                vel = np.sqrt(state_dict[ind]['u'] ** 2 + state_dict[ind]['v'] ** 2)
                prev_vel = np.sqrt(state_dict[ind]['prev_u'] ** 2 + state_dict[ind]['prev_v'] ** 2)
                while abs(vel - prev_vel) > 1e-2 and index < iterations:
                    start = timeit.default_timer()
                    plt.figure(figsize=(16, 9), dpi=160)
                    force = SpatialPlacement.calculate_forces(state_dict, work_piece_copy, 1000.0, spring_const=50e4)

                    num_coll = SpatialPlacement.find_collisions(state_dict, work_piece)
                    dt = dt_0 * np.exp(-vel / 950.0)
                    SpatialPlacement.integrate_state_dict(state_dict, dt, force, work_piece_copy)
                    SpatialPlacement.apply_collisions(state_dict, work_piece)
                    prim.GeometricFunctions.plot_cross_sections_on_workpiece(state_dict, work_piece_copy, output_dir,
                                                                             index='%s_%s' % (ind, index), new_fig=False)
                    # work_piece_copy = prim.WorkPiece(width=work_piece_copy.width*0.99, height=work_piece_copy.height*0.99,
                    #                                  thickness=work_piece.thickness)
                    index += 1
                    used_work_space = SpatialPlacement.used_workpiece_area(state_dict)
                    logger.info('Finished iteration %s, took %ss, DT: %ss', index, timeit.default_timer() - start, dt)
                    logger.info('Space Usage %smm^2, Change %smm^2', used_work_space, (used_work_space - prev_used_space))
                    prev_used_space = used_work_space
                    plt.close('all')
                    vel = np.sqrt(state_dict[ind]['u'] ** 2 + state_dict[ind]['v'] ** 2)
                    prev_vel = np.sqrt(state_dict[ind]['prev_u'] ** 2 + state_dict[ind]['prev_v'] ** 2)

                ind += 1
            plt.figure()
            plt.subplot(1, 2, 1)
            plt.plot(list(range(0, len(dt_lst))), dt_lst)
            plt.title('DT Vs Iteration')
            plt.subplot(1, 2, 2)
            plt.plot(list(range(0, len(dt_lst))), max_force_lst)
            plt.title('Max Force vs Iteration')
            plt.savefig(os.path.join(output_dir, 'meta_data.png'))

            logger.info("Performing the final translation of all CrossSectionPairs.")
            for ind in range(len(state_dict)):
                prim.GeometricFunctions.move_cross_section_from_state_dict(
                    path_1=state_dict[ind]['section'].section1.get_path(),
                    path_2=state_dict[ind]['section'].section2.get_path(),
                    dict_entry=state_dict[ind])

            logger.info("Plotting the CrossSectionPairs on the work_piece for visualization.")
            prim.GeometricFunctions.plot_cross_sections_on_workpiece(state_dict, work_piece, output_dir, index='final')

        @staticmethod
        def align_cross_sections_on_workpiece(cross_section_list, work_piece, output_dir, wire_cutter,
                                              distance_between_sections):
            """

            :param list[comp.CrossSectionPair] cross_section_list:
            :param prim.WorkPiece work_piece:
            :param WireCutter wire_cutter:
            :return:
            """
            # SpatialPlacement.align_cross_sections_on_workpiece_single(cross_section_list, work_piece, output_dir, wire_cutter,
            #                                                    distance_between_sections)
            # pass
            logger = logging.getLogger(__name__)
            state_dict = dict()
            ind = 0
            for section in cross_section_list:
                state_dict[ind] = {'section': section, 'rot': 0, 'omega': 0, 'x_pos': 0, 'y_pos': 0, 'v': 10, 'u': -10,
                                   'prev_rot': 0, 'prev_omega': 0, 'prev_x': 0, 'prev_y': 0, 'prev_v': 10, 'prev_u': -10,
                                   'collider': SpatialPlacement.create_section_collider(section, distance_between_sections),
                                   'collisions': [], 'prev_collisions': []}
                ind += 1
            logger.info("Setting initial positions for Cross Sections on the work_piece")
            SpatialPlacement.set_initial_cross_section_offsets(state_dict, work_piece)
            prim.GeometricFunctions.plot_cross_sections_on_workpiece(state_dict, work_piece, output_dir, index='initial')
            plt.show()

            prev_used_space = SpatialPlacement.used_workpiece_area(state_dict)

            index = 0
            dt_0 = 0.1
            dt = dt_0
            iterations = 10000
            max_force_lst = list()
            dt_lst = list()
            work_piece_copy = copy.deepcopy(work_piece)
            while index < iterations:
                start = timeit.default_timer()
                plt.figure(figsize=(16, 9), dpi=160)
                force = SpatialPlacement.calculate_forces(state_dict, work_piece_copy, 1000.0, spring_const=50e4)
                prev_dt = 0
                num_collision = SpatialPlacement.find_collisions(state_dict, work_piece)
                SpatialPlacement.integrate_state_dict(state_dict, dt, force, work_piece_copy)
                num_collision = SpatialPlacement.find_collisions(state_dict, work_piece)
                # Search for collisions through time until there is only one collision at each integration step
                while num_collision > 1:
                    SpatialPlacement.clear_collisions(state_dict)
                    SpatialPlacement.backtrack_state_dict(state_dict)
                    prev_dt = dt
                    dt = prev_dt / 2
                    if dt < 1e-8:
                        logger.debug('Collisions at break: %s', num_collision)
                        break
                dt = dt_0
                SpatialPlacement.apply_collisions(state_dict, work_piece)
                prim.GeometricFunctions.plot_cross_sections_on_workpiece(state_dict, work_piece_copy, output_dir,
                                                                         index=index, new_fig=False)
                # work_piece_copy = prim.WorkPiece(width=work_piece_copy.width*0.99, height=work_piece_copy.height*0.99,
                #                                  thickness=work_piece.thickness)
                index += 1
                used_work_space = SpatialPlacement.used_workpiece_area(state_dict)
                logger.info('Finished iteration %s, took %ss', index, timeit.default_timer() - start)
                logger.info('Space Usage %smm^2, Change %smm^2', used_work_space, (used_work_space - prev_used_space))
                prev_used_space = used_work_space
                plt.close('all')

            plt.figure()
            plt.subplot(1, 2, 1)
            plt.plot(list(range(0, len(dt_lst))), dt_lst)
            plt.title('DT Vs Iteration')
            plt.subplot(1, 2, 2)
            plt.plot(list(range(0, len(dt_lst))), max_force_lst)
            plt.title('Max Force vs Iteration')
            plt.savefig(os.path.join(output_dir, 'meta_data.png'))

            logger.info("Performing the final translation of all CrossSectionPairs.")
            for ind in state_dict:
                prim.GeometricFunctions.move_cross_section_from_state_dict(
                    path_1=state_dict[ind]['section'].section1.get_path(),
                    path_2=state_dict[ind]['section'].section2.get_path(),
                    dict_entry=state_dict[ind])

            logger.info("Plotting the CrossSectionPairs on the work_piece for visualization.")
            prim.GeometricFunctions.plot_cross_sections_on_workpiece(state_dict, work_piece, output_dir, index='final')

        @staticmethod
        def clear_collisions(state_dict):
            for ind in state_dict:
                state_dict[ind]['collisions'] = list()

        @staticmethod
        def apply_collisions(state_dict, work_piece, e=0.75):
            logger = logging.getLogger(__name__)
            for ind in state_dict:
                if state_dict[ind]['collisions']:
                    for ind2 in range(len(state_dict[ind]['collisions'])):
                        state_dict_2 = state_dict[ind]['collisions'][ind2][0]
                        path1 = state_dict[ind]['collider'].get_path()
                        center1 = prim.GeometricFunctions.get_center_of_path(path1)

                        if isinstance(state_dict[ind]['collisions'][ind2][0], str):
                            point = state_dict[ind]['collisions'][ind2][1]
                            r = point - center1
                            r1 = [r['x'], r['y'], r['z']]
                            vel = [state_dict[ind]['u'], state_dict[ind]['v'], 0]
                            omega = [0, 0, state_dict[ind]['omega']]
                            vr = SpatialPlacement.point_velocity(vel, omega, r1)

                            if point['x'] < 0:
                                if np.sign(state_dict[ind]['u']) == -1:
                                    n = [1, 0, 0]
                                    j = SpatialPlacement.reaction_impulse_mag(state_dict[ind]['mass'], 1e12,
                                                                              state_dict[ind]['I'],
                                                                              1e12, r1=r1, r2=[0, 0, 0], e=e * 0.5, n=n,
                                                                              vr=vr)
                                    state_dict[ind]['u'] = state_dict[ind]['u'] + j * n[0] / state_dict[ind]['mass']
                                    state_dict[ind]['omega'] = state_dict[ind]['omega'] + j * np.cross(r1, n)[2] / \
                                                               state_dict[ind]['I']
                            if point['x'] > work_piece.width:
                                if np.sign(state_dict[ind]['u']) == 1:
                                    n = [-1, 0, 0]
                                    j = SpatialPlacement.reaction_impulse_mag(state_dict[ind]['mass'], 1e12,
                                                                              state_dict[ind]['I'],
                                                                              1e12, r1=r1, r2=[0, 0, 0], e=e * 0.5, n=n,
                                                                              vr=vr)
                                    state_dict[ind]['u'] = state_dict[ind]['u'] + j * n[0] / state_dict[ind]['mass']
                                    state_dict[ind]['omega'] = state_dict[ind]['omega'] + j * np.cross(r1, n)[2] / \
                                                               state_dict[ind]['I']

                            if point['y'] < 0:
                                if np.sign(state_dict[ind]['v']) == -1:
                                    n = [0, 1, 0]
                                    j = SpatialPlacement.reaction_impulse_mag(state_dict[ind]['mass'], 1e12,
                                                                              state_dict[ind]['I'],
                                                                              1e12, r1=r1, r2=[0, 0, 0], e=e * 0.5, n=n,
                                                                              vr=vr)
                                    state_dict[ind]['v'] = state_dict[ind]['v'] + j * n[1] / state_dict[ind]['mass']
                                    state_dict[ind]['omega'] = state_dict[ind]['omega'] + j * np.cross(r1, n)[2] / \
                                                               state_dict[ind]['I']
                            if point['y'] > work_piece.height:
                                if np.sign(state_dict[ind]['v']) == 1:
                                    n = [0, -1, 0]
                                    j = SpatialPlacement.reaction_impulse_mag(state_dict[ind]['mass'], 1e12,
                                                                              state_dict[ind]['I'],
                                                                              1e12, r1=r1, r2=[0, 0, 0], e=e * 0.5, n=n,
                                                                              vr=vr)
                                    state_dict[ind]['v'] = state_dict[ind]['v'] + j * n[1] / state_dict[ind]['mass']
                                    state_dict[ind]['omega'] = state_dict[ind]['omega'] + j * np.cross(r1, n)[2] / \
                                                               state_dict[ind]['I']

                        else:
                            path2 = state_dict_2['collider'].get_path()
                            center2 = prim.GeometricFunctions.get_center_of_path(path2)
                            r1 = (state_dict[ind]['collisions'][ind2][2] - center1).as_ndarray()
                            r2 = (state_dict[ind]['collisions'][ind2][2] - center2).as_ndarray()
                            n = state_dict[ind]['collisions'][ind2][1].normal()

                            vel1 = np.array([state_dict[ind]['u'], state_dict[ind]['v'], 0])
                            vel2 = np.array([state_dict_2['u'], state_dict_2['v'], 0])

                            vp1 = SpatialPlacement.point_velocity(vel1, [0, 0, state_dict[ind]['omega']], r1)
                            vp2 = SpatialPlacement.point_velocity(vel2, [0, 0, state_dict_2['omega']], r2)

                            impulse_mag = SpatialPlacement.reaction_impulse_mag(state_dict[ind]['mass'],
                                                                                state_dict_2['mass'],
                                                                                state_dict[ind]['I'], state_dict_2['I'], r1,
                                                                                r2,
                                                                                e=e, n=n,
                                                                                vr=vp1 - vp2)

                            state_dict[ind]['u'] = state_dict[ind]['u'] + impulse_mag * n[0] / state_dict[ind]['mass']
                            state_dict[ind]['v'] = state_dict[ind]['v'] + impulse_mag * n[1] / state_dict[ind]['mass']

                            state_dict_2['u'] = state_dict_2['u'] - impulse_mag * n[0] / state_dict_2['mass']
                            state_dict_2['v'] = state_dict_2['v'] - impulse_mag * n[1] / state_dict_2['mass']

                            state_dict[ind]['omega'] = state_dict[ind]['omega'] + impulse_mag * np.cross(r1, n)[2] / \
                                                       state_dict[ind]['I']
                            state_dict_2['omega'] = state_dict_2['omega'] - impulse_mag * np.cross(r1, n)[2] / state_dict_2[
                                'I']

                    state_dict[ind]['prev_collisions'] = state_dict[ind]['collisions']
                    state_dict[ind]['collisions'] = []

        @staticmethod
        def used_workpiece_area(state_dict):
            """

            :param dict state_dict:
            :return:
            """
            max_x = 0
            max_y = 0
            min_x = sys.maxsize
            min_y = sys.maxsize
            for item in state_dict.values():
                path = item['collider'].get_path()
                tmp_max_x = prim.GeometricFunctions.get_point_from_max_coord(path, 'x')[1]
                tmp_max_y = prim.GeometricFunctions.get_point_from_max_coord(path, 'y')[1]
                tmp_min_x = prim.GeometricFunctions.get_point_from_min_coord(path, 'x')['x']
                tmp_min_y = prim.GeometricFunctions.get_point_from_min_coord(path, 'y')['y']

                if tmp_max_x > max_x:
                    max_x = tmp_max_x
                if tmp_max_y > max_y:
                    max_y = tmp_max_y
                if tmp_min_y < min_y:
                    min_y = tmp_min_y
                if tmp_min_x < min_x:
                    min_x = tmp_min_x
            return (max_x - min_x) * (max_y - min_y)

        @staticmethod
        def backtrack_state_dict(state_dict):
            for val in state_dict.values():
                delta_x = val['prev_x'] - val['x_pos']
                delta_y = val['prev_y'] - val['y_pos']
                delta_rot = np.deg2rad(val['prev_rot'] - val['rot'])

                spma.PointManip.Transform.rotate(val['collider'].get_path(), [0, 0, delta_rot],
                                                 origin=[val['x_pos'], val['y_pos'], 0])
                val['collider'].translate([delta_x, delta_y, 0])

                val['x_pos'] = val['prev_x']
                val['y_pos'] = val['prev_y']
                val['u'] = val['prev_u']
                val['v'] = val['prev_v']
                val['rot'] = val['prev_rot']
                val['omega'] = val['prev_omega']

        @staticmethod
        def integrate_state_dict(state_dict, dt, force, work_piece):
            # Simple euler integration for now, can switch to RG4 if there are stability issues.
            for ind1, val1 in state_dict.items():
                val1['prev_u'] = val1['u']
                val1['prev_v'] = val1['v']
                val1['prev_x'] = val1['x_pos']
                val1['prev_y'] = val1['y_pos']
                val1['prev_rot'] = val1['rot']
                val1['prev_omega'] = val1['omega']

                delta_x = val1['u'] * dt
                delta_y = val1['v'] * dt
                delta_rot = val1['omega'] * dt

                val1['collider'].translate([delta_x, delta_y, 0])
                val1['x_pos'] += delta_x
                val1['y_pos'] += delta_y

                spma.PointManip.Transform.rotate(val1['collider'].get_path(), [0, 0, delta_rot],
                                                 origin=[val1['x_pos'], val1['y_pos'], 0])
                val1['rot'] += np.rad2deg(delta_rot)

                val1['u'] += force[ind1, 0] * dt
                val1['v'] += force[ind1, 1] * dt
                mag_vel = np.sqrt(val1['u'] ** 2 + val1['v'] ** 2)
                if mag_vel > SpatialPlacement.MAX_VEL:
                    val1['u'] = val1['u'] / mag_vel * SpatialPlacement.MAX_VEL
                    val1['v'] = val1['v'] / mag_vel * SpatialPlacement.MAX_VEL
                val1['omega'] += force[ind1, 2] * dt / 1000.0

        @staticmethod
        def calculate_forces(state_dict, work_piece, beta, spring_const):
            """

            :param dict[int, dict] state_dict:
            :param prim.WorkPiece work_piece:
            :return:
            """
            force = np.zeros([len(state_dict), 3])  # fx, fy, tau
            for ind in state_dict:
                force[ind, 0] += -beta * 2
                force[ind, 1] += beta

            return force

        @staticmethod
        def point_velocity(v_body, angular_velocity, vector_to_point):
            """

            :param v_body:
            :param angular_velocity:
            :param vector_to_point:
            :return:
            """
            return v_body + np.cross(angular_velocity, vector_to_point)

        @staticmethod
        def reaction_impulse_mag(m1, m2, i1, i2, r1, r2, e, n, vr):
            """
            Calculates the reaction impulse magnitude of two colliding objects.

            :param m1: Mass of object 1 (kg),
            :param m2: Mass of object 2 (kg).
            :param i1: Moment of Inertia of object 1.
            :param i2: Moment of Inertia of object 2.
            :param r1: Vector from center of object 1 to the point of collision.
            :param r2: Vector from center of object 2 to the point of collision.
            :param e: Coefficient of restitution. [0...1]
            :param n: Normal vector from object 1 to object 2 at point of contact.
            :param vr: Relative velocity of the two objects at collision
            :return: Magnitude of the reaction impulse.
            """
            numerator = -np.dot((1 + e) * vr, n)
            denominator = m1 ** -1 + m2 ** -1 + np.dot(((i1 ** -1) * np.cross((np.cross(r1, n)), r1) +
                                                        (i2 ** -1) * np.cross((np.cross(r2, n)), r2)), n)

            return numerator / denominator

        @staticmethod
        def find_collisions(state_dict, work_piece):
            num_collisisons = 0
            for ind in state_dict:
                path1 = state_dict[ind]['collider'].get_path()
                for point in path1:
                    if point['x'] < 0:
                        state_dict[ind]['collisions'].append(['wall', point])
                    elif point['x'] > work_piece.width:
                        state_dict[ind]['collisions'].append(['wall', point])

                    if point['y'] < 0:
                        state_dict[ind]['collisions'].append(['wall', point])
                    elif point['y'] > work_piece.height:
                        state_dict[ind]['collisions'].append(['wall', point])

                bb1 = prim.GeometricFunctions.get_bounding_box_from_path(path1)
                for ind2 in range(0, len(state_dict) - 1):
                    path2 = state_dict[ind2]['collider'].get_path()
                    bb2 = prim.GeometricFunctions.get_bounding_box_from_path(path2)
                    if prim.GeometricFunctions.bounding_boxes_intersect(bb1, bb2):
                        for point in path1:
                            if state_dict[ind2]['collider'].point_in_section(point):
                                closest_line = prim.Line.line_from_points(path2[0], path2[1])
                                dist = abs(closest_line.signed_distance_to_point_xy(point))
                                for ind3 in range(1, len(path2) - 1):
                                    line2 = prim.Line.line_from_points(path2[ind3], path2[ind3 + 1])
                                    tmp_dist = abs(line2.signed_distance_to_point_xy(point))
                                    if tmp_dist < dist:
                                        closest_line = line2
                                        dist = tmp_dist
                                add_col = True
                                for col in state_dict[ind]['prev_collisions']:
                                    if state_dict[ind2] == col[0]:
                                        add_col = False
                                        break
                                r = (np.array([state_dict[ind]['x_pos'], state_dict[ind]['y_pos'], 0]) - np.array(
                                    [state_dict[ind2]['x_pos'], state_dict[ind2]['y_pos'], 0]))
                                vel = (np.array([state_dict[ind]['u'], state_dict[ind]['v'], 0]) - np.array(
                                    [state_dict[ind2]['u'], state_dict[ind2]['v'], 0]))

                                dot = np.dot(r, vel)
                                if np.sign(dot) == 1:
                                    add_col = False
                                if add_col:
                                    state_dict[ind]['collisions'].append([state_dict[ind2], closest_line, point])
                                    num_collisisons += 1
                                break
            return num_collisisons

        @staticmethod
        def apply_collision_forces(state_dict, force, work_piece, spring_const):
            for ind1 in range(len(state_dict)):
                bb1 = prim.GeometricFunctions.get_bounding_box_from_path(state_dict[ind1]['collider'].get_path())
                for ind2 in range(ind1 + 1, len(state_dict)):
                    bb2 = prim.GeometricFunctions.get_bounding_box_from_path(state_dict[ind2]['collider'].get_path())
                    if prim.GeometricFunctions.bounding_boxes_intersect(bb1, bb2):
                        force1, force2 = SpatialPlacement.calculate_collision_force(state_dict[ind1]['collider'],
                                                                                    state_dict[ind2]['collider'],
                                                                                    spring_const=spring_const)
                        force[ind1, :] += force1
                        force[ind2, :] += force2

                if bb1[0][0] < 0 or bb1[1][0] > work_piece.width or bb1[0][1] < 0 or bb1[1][1] > work_piece.height:
                    SpatialPlacement.calculate_wall_collision_force(state_dict[ind1], work_piece, force)

        @staticmethod
        def calculate_collision_force(collider1, collider2, spring_const):
            """

            :param comp.CrossSection collider1:
            :param comp.CrossSection collider2:
            :param float spring_const:
            :return:
            """
            force1 = np.zeros(3)
            force2 = np.zeros(3)
            center1 = prim.Point(0, 0, 0)
            center2 = prim.Point(0, 0, 0)
            path1 = collider1.get_path()
            path2 = collider2.get_path()

            for point in path1:
                center1 += point
            for point in path2:
                center2 += point
            center1 /= len(path1)
            center2 /= len(path2)

            for point in path1:
                if collider2.point_in_section(point):
                    path2 = collider2.get_path()
                    closest_line = prim.Line.line_from_points(path2[0], path2[1])
                    dist = abs(closest_line.signed_distance_to_point_xy(point))
                    for ind in range(1, len(path2) - 1):
                        line2 = prim.Line.line_from_points(path2[ind], path2[ind + 1])
                        tmp_dist = abs(line2.signed_distance_to_point_xy(point))
                        if tmp_dist < dist:
                            closest_line = line2
                            dist = tmp_dist

                    SpatialPlacement.get_collision_force_from_line(center1, center2, point, closest_line,
                                                                   force1, force2, dist, spring_const)
            # prim.GeometricFunctions.plot_path(path1, color='b')
            # prim.GeometricFunctions.plot_path(path2, color='k')
            # plt.show()
            return force1, force2

        @staticmethod
        def get_collision_force_from_line(center1, center2, point, line, force1, force2, dist, spring_const):
            normal_vecs = prim.GeometricFunctions.normal_vector(line.get_path()[0], line.get_path()[1], next_point=None)[0]
            force = spring_const * dist
            force_x = force * normal_vecs[0]
            force_y = force * normal_vecs[1]
            force1[0] += force_x
            force1[1] += force_y
            moment1 = force_x * (point['y'] - center1['y']) - force_y * (point['x'] - center1['x'])
            force1[2] += moment1
            color = 'g' if moment1 > 0 else 'r'
            plt.quiver([point['x']], [point['y']], [force_x], [force_y], width=0.001, color=color)

            force2[0] += -force_x
            force2[1] += -force_y
            moment2 = (-force_x * (point['y'] - center2['y']) +
                       force_y * (point['x'] - center2['x']))
            force2[2] += moment2
            color = 'g' if moment2 > 0 else 'r'
            plt.quiver([point['x']], [point['y']], [-force_x], [-force_y], width=0.001,
                       color=color)

        @staticmethod
        def get_collision_force_from_points(center1, center2, point1, point2, force1, force2, dist, spring_const):
            direction = point2 - point1
            direction = np.array([direction['x'], direction['y']])
            unit_vector = direction / dist
            force = spring_const * dist
            force_x = force * unit_vector[0]
            force_y = force * unit_vector[1]
            force1[0] += force_x
            force1[1] += force_y
            moment1 = force_x * (point1['y'] - center1['y']) - force_y * (point1['x'] - center1['x'])
            force1[2] += moment1
            color = 'g' if moment1 > 0 else 'r'
            plt.quiver([point1['x']], [point1['y']], [force_x], [force_y], width=0.001, color=color)

            force2[0] += -force_x
            force2[1] += -force_y
            moment2 = (-force_x * (point2['y'] - center2['y']) +
                       force_y * (point2['x'] - center2['x']))
            force2[2] += moment2
            color = 'g' if moment2 > 0 else 'r'
            plt.quiver([point2['x']], [point2['y']], [-force_x], [-force_y], width=0.001,
                       color=color)

        @staticmethod
        def calculate_wall_collision_force(state_dict_entry, work_piece, force):
            path = state_dict_entry['collider'].get_path()
            center = prim.Point(0, 0, 0)

            for point in path:
                center += point
            center /= len(path)

            x_handled = False
            y_handled = False

            for point in path:
                if point['x'] < 0 and not x_handled:
                    tmp_force = abs(force[0])
                    force[0] = 0
                    force[2] += tmp_force * (center['y'] - point['y'])
                    state_dict_entry['v'] = 0
                    x_handled = True
                elif point['x'] > work_piece.width and not x_handled:
                    tmp_force = abs(force[0])
                    force[0] = 0
                    force[2] += -tmp_force * (center['y'] - point['y'])
                    state_dict_entry['v'] = 0
                    x_handled = True

                if point['y'] < 0 and not y_handled:
                    tmp_force = abs(force[1])
                    force[1] = 0
                    force[2] += -tmp_force * (center['x'] - point['x'])
                    state_dict_entry['u'] = 0
                    y_handled = True
                elif point['y'] > work_piece.height and not y_handled:
                    tmp_force = abs(force[1])
                    force[1] = 0
                    force[2] += -tmp_force * (center['x'] - point['x'])
                    state_dict_entry['u'] = 0
                    y_handled = True

        @staticmethod
        def move_state_dict_from_potential_field(state_dict, potential_field, dt, x_grid, y_grid, work_piece):
            """

            :param dict[int, dict] state_dict:
            :param np.ndarray potential_field:
            :param float dt:
            :param x_grid:
            :param y_grid:
            :param prim.WorkPiece work_piece:
            :return:
            """
            logger = logging.getLogger(__name__)
            for ind in state_dict:
                center = prim.Point(0, 0, 0)
                path = state_dict[ind]['collider'].get_path()
                x_force = 0
                y_force = 0
                torque = 0
                for point in path:
                    center += point
                center /= len(path)

                for point in path:
                    x = point['x']
                    y = point['y']
                    x_coord = np.argmin(np.abs(x_grid - x))
                    y_coord = np.argmin(np.abs(y_grid - y))

                    force = potential_field[x_coord, y_coord, :]
                    x_force += force[0]
                    y_force += force[1]
                    lever_arm = point - center
                    torque += lever_arm['x'] * force[1] + lever_arm['y'] * force[0]

                # If the segment is outside of the work_piece, add additional force to bring it back in.
                if state_dict[ind]['x_pos'] < 0:
                    x_force += 20
                elif state_dict[ind]['x_pos'] > work_piece.width:
                    x_force -= 20
                if state_dict[ind]['y_pos'] < 0:
                    y_force += 20
                elif state_dict[ind]['y_pos'] > work_piece.height:
                    y_force -= 20

                delta_x = 100 * (x_force * dt ** 2) / 2  # Mass is assumed 1/100
                delta_y = 100 * (y_force * dt ** 2) / 2  # Mass is assumed 1/100
                delta_rot = 0.2 * (torque * dt ** 2) / 2  # Moment of Inertia is assumed 4

                # logger.debug('delta_x : %s, delta_y: %s, delta_rot: %s', delta_x, delta_y, np.rad2deg(delta_rot))

                state_dict[ind]['collider'].translate([delta_x, delta_y, 0])
                state_dict[ind]['x_pos'] += delta_x
                state_dict[ind]['y_pos'] += delta_y

                spma.PointManip.Transform.rotate(path, [0, 0, delta_rot],
                                                 origin=[state_dict[ind]['x_pos'], state_dict[ind]['y_pos'], 0])

                state_dict[ind]['rot'] += np.rad2deg(delta_rot)

        @staticmethod
        def calculate_potential_field(x_mesh, y_mesh, state_dict, alpha, beta, field, work_piece):
            logger = logging.getLogger(__name__)
            for ind_x in range(0, x_mesh.shape[0]):
                start = timeit.default_timer()
                for ind_y in range(0, y_mesh.shape[1]):
                    x = x_mesh[ind_x, ind_y]
                    y = y_mesh[ind_x, ind_y]

                    closest_sections = SpatialPlacement.get_n_closest_sections(x=x, y=y, state_dict=state_dict, n=3)
                    for section in closest_sections:
                        field[ind_x, ind_y, :] += alpha * SpatialPlacement.calculate_force_for_potential_field(x, y,
                                                                                                               section,
                                                                                                               work_piece,
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
            closest_dist = np.sqrt((prim.Point(closest_point['x'], closest_point['y'], 0) - curr_point) ** 2)
            for point in section.get_path():
                dist = np.sqrt((prim.Point(point['x'], point['y'], 0) - curr_point) ** 2)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_point = point

            resultant_vector = closest_point - curr_point
            dist = np.sqrt(resultant_vector ** 2)
            if section.point_in_section(prim.Point(x, y, 0)):
                scale = abs((2 * dist) ** 1.5) / 10.0 * beta  # forcing value to get points outside of other collider box
                force[0] = scale * resultant_vector[0]
                force[1] = scale * resultant_vector[1]
            else:
                scale = np.log10(abs(dist / 10.0) ** 1.5) / 10.0 * beta
                force[0] = scale * resultant_vector[0]
                force[1] = scale * resultant_vector[1]

            # Add constant external force to move cross sections towards the upper left
            force[0] += -8 * (x / 100)
            force[1] += 4 * ((work_piece.height - y) / 100)

            SpatialPlacement.add_wall_force(force, x=x, y=y, work_piece=work_piece, beta=6, wall_dist=25.0)

            return force

        @staticmethod
        def add_wall_force(force, x, y, work_piece, beta, wall_dist):
            if abs(x - work_piece.width) < wall_dist:
                force[0] += -max([15 * beta / abs(x - work_piece.width), wall_dist])
            elif abs(x) < wall_dist:
                force[0] += max([15 * beta / abs(x - work_piece.width), wall_dist])

            if abs(y - work_piece.height) < wall_dist:
                force[1] += -max([5.0 * beta / abs(y - work_piece.height), wall_dist])
            elif abs(y) < wall_dist:
                force[1] += max([5.0 * beta / abs(y - work_piece.height), wall_dist])

        @staticmethod
        def get_n_closest_sections(x, y, state_dict, n):
            section_list = list()
            dist_list = list()
            for key in state_dict:
                dist = np.sqrt((x - state_dict[key]['x_pos']) ** 2 + (y - state_dict[key]['y_pos']) ** 2)
                if len(section_list) < n:
                    section_list.append(state_dict[key]['collider'])
                    dist_list.append(dist)
                else:
                    max_dist = max(dist_list)
                    if max_dist > dist:
                        max_index = dist_list.index(max_dist)
                        dist_list[max_index] = dist
                        section_list[max_index] = state_dict[key]['collider']

            return section_list

        @staticmethod
        def set_initial_cross_section_offsets(state_dict, work_piece):
            """

            :param state_dict:
            :param prim.WorkPiece work_piece:
            :return:
            """
            max_x = work_piece.width
            max_y = work_piece.height
            bb = prim.GeometricFunctions.get_bounding_box_from_path(state_dict[0]['collider'].get_path())
            curr_x = (bb[1]['x'] - bb[0]['x']) * 1.6 / 2.0
            curr_y = max_y - (bb[1]['y'] - bb[0]['y']) * 1.6 / 2.0
            # curr_x = work_piece.width*0.8
            # curr_y = work_piece.height*0.2

            for ind in range(len(state_dict)):
                state_dict[ind]['x_pos'] = curr_x
                state_dict[ind]['y_pos'] = curr_y
                prim.GeometricFunctions.center_path(state_dict[ind]['collider'].get_path())
                state_dict[ind]['collider'].translate([curr_x, curr_y, 0])

                prim.GeometricFunctions.center_path(state_dict[ind]['collider'].get_path())
                bound_box_curr = prim.GeometricFunctions.get_bounding_box_from_path(
                    state_dict[ind]['collider'].get_path())

                if bound_box_curr[1][0] - bound_box_curr[0][0] < bound_box_curr[1][1] - bound_box_curr[0][1]:
                    state_dict[ind]['rot'] = 90.0
                    spma.PointManip.Transform.rotate(state_dict[ind]['collider'].get_path(),
                                                     [0, 0, np.deg2rad(state_dict[ind]['rot'])])

                    bound_box_curr = prim.GeometricFunctions.get_bounding_box_from_path(
                        state_dict[ind]['collider'].get_path())
                if ind > 1:
                    bound_box_last = prim.GeometricFunctions.get_bounding_box_from_path(
                        state_dict[ind - 1]['collider'].get_path())

                    curr_x += (bound_box_last[1]['x'] - bound_box_last[0]['x']) * 1.6 / 2.0 + \
                              (bound_box_curr[1]['x'] - bound_box_curr[0]['x']) * 1.6 / 2.0

                    if curr_x + (bound_box_curr[1]['x'] - bound_box_curr[0]['x']) / 2 >= work_piece.width:
                        curr_y -= (bound_box_last[1]['y'] - bound_box_last[0]['y']) * 1.6 / 2 + \
                                  (bound_box_curr[1]['y'] - bound_box_curr[0]['y']) * 1.6 / 2
                        curr_x = (bound_box_curr[1]['x'] - bound_box_curr[0]['x']) * 1.6 / 2.0

                state_dict[ind]['x_pos'] = curr_x
                state_dict[ind]['y_pos'] = curr_y

                state_dict[ind]['collider'].translate([curr_x, curr_y, 0])
