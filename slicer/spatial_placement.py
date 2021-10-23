import logging
import os

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

    def __init__(self, work_piece, wire_cutter, vert_spacing=1):
        self.logger = logging.getLogger(__name__)
        self.state_dict = dict()
        self.r_index = [rtree.index.Index()]
        self.work_piece = work_piece
        self.section_order = list()
        self.unplaced_ind = list()
        self.wire_cutter = wire_cutter
        self.num_sections = 0
        self.vert_spacing = vert_spacing

    def bin_packing_algorithm(self, cross_section_list, output_dir, distance_between_sections):
        ind = 0
        self.num_sections = 1
        for index, section in enumerate(cross_section_list):
            path_1 = section.get_path()[0]
            path_2 = section.get_path()[1]

            # plt.close('all')
            # plt.figure(figsize=(16, 9), dpi=320)
            # prim.GeometricFunctions.plot_path(path_1, color='C1')
            # prim.GeometricFunctions.plot_path(path_2, color='C2')
            # plt.axis('equal')
            # plt.savefig(os.path.join(output_dir, 'Subdivide_before_center_%s.png' % (index)))
            # plt.show()

            center = prim.Point(0, 0, 0)
            for point in path_1:
                center += point
            for point in path_2:
                center += point
            center /= (len(path_1) + len(path_2))
            spma.PointManip.Transform.translate(path_1, -center)
            spma.PointManip.Transform.translate(path_2, -center)

            # plt.close('all')
            # plt.figure(figsize=(16, 9), dpi=320)
            # prim.GeometricFunctions.plot_path(path_1, color='C1')
            # prim.GeometricFunctions.plot_path(path_2, color='C2')
            # plt.axis('equal')
            # plt.savefig(os.path.join(output_dir, 'Subdivide_after_center_%s.png' % (index)))
            # plt.show()

            collider1, collider2 = SpatialPlacement.create_section_collider(section, distance_between_sections)
            bb = prim.GeometricFunctions.get_bounding_box_from_path(collider1.get_path() + collider2.get_path())
            bb_center = prim.Point(x=(bb[1]['x'] + bb[0]['x']) / 2, y=(bb[1]['y'] + bb[0]['y']) / 2, z=0)
            path_center = prim.GeometricFunctions.get_center_of_path(collider1.get_path() + collider2.get_path())
            diff_centers = path_center - bb_center

            ang_from_centers = np.arctan2(diff_centers['y'], diff_centers['x'])
            ang_from_centers = ang_from_centers + 2 * np.pi if ang_from_centers < 0 else ang_from_centers
            rot = -ang_from_centers

            self.state_dict[ind] = {'section': section, 'rot': rot, 'x_pos': 0, 'y_pos': 0,
                                    'collider1': collider1, 'collider2': collider2, 'bb': bb, 'cut': 0}
            spma.PointManip.Transform.rotate(collider1.get_path(), [0, 0, np.deg2rad(rot)], origin=path_center)
            spma.PointManip.Transform.rotate(collider2.get_path(), [0, 0, np.deg2rad(rot)], origin=path_center)

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

        self.calculate_initial_rotation()

        r_id = 0

        prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
                                                                 num_sections=self.num_sections, index='initial')

        last_pos_x = 0
        last_pos_y = self.work_piece.height
        indx = 0
        time_out = 10000

        for ind in self.state_dict:
            item = self.state_dict[ind]
            placed = False
            iterations = 0
            while not placed:
                delta_x = last_pos_x - item['x_pos']
                delta_y = last_pos_y - item['y_pos']
                spma.PointManip.Transform.translate(item['collider1'].get_path(), [delta_x, delta_y, 0])
                spma.PointManip.Transform.translate(item['collider2'].get_path(), [delta_x, delta_y, 0])
                item['x_pos'] = last_pos_x
                item['y_pos'] = last_pos_y
                bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider1'].get_path() +
                                                                             item['collider2'].get_path())

                # Make sure the current bounding box is not outside of the work piece, we check against the
                # top right corner first, then check quad tree intersections
                if bb_curr[0]['x'] < 0:
                    delta_x = -bb_curr[0]['x']
                    spma.PointManip.Transform.translate(item['collider1'].get_path(), [delta_x, 0, 0])
                    spma.PointManip.Transform.translate(item['collider2'].get_path(), [delta_x, 0, 0])
                    item['x_pos'] += delta_x
                    last_pos_x += delta_x
                    bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider1'].get_path() +
                                                                                 item['collider2'].get_path())
                if bb_curr[1]['y'] > self.work_piece.height:
                    delta_y = self.work_piece.height - bb_curr[1]['y']
                    spma.PointManip.Transform.translate(item['collider1'].get_path(), [0, delta_y, 0])
                    spma.PointManip.Transform.translate(item['collider2'].get_path(), [0, delta_y, 0])
                    item['y_pos'] += delta_y
                    last_pos_y += delta_y
                    bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider1'].get_path() +
                                                                                 item['collider2'].get_path())
                (left, bottom, right, top) = bb_curr[0]['x'], bb_curr[0]['y'], bb_curr[1]['x'], bb_curr[1]['y']
                intersections = list(self.r_index[self.num_sections - 1].intersection((left, bottom, right, top)))
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
                            spma.PointManip.Transform.translate(item['collider1'].get_path(), [0, delta_y, 0])
                            spma.PointManip.Transform.translate(item['collider2'].get_path(), [0, delta_y, 0])
                            bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider1'].get_path() +
                                                                                         item['collider2'].get_path())
                            last_pos_y += delta_y
                            item['y_pos'] += delta_y

                            (left, bottom, right, top) = bb_curr[0]['x'], bb_curr[0]['y'], bb_curr[1]['x'], bb_curr[1][
                                'y']
                            intersections = list(
                                self.r_index[self.num_sections - 1].intersection((left, bottom, right, top)))
                            if len(intersections) > 0 and bb_curr[0]['y'] > 0:
                                delta_y = -1
                                spma.PointManip.Transform.translate(item['collider1'].get_path(), [0, delta_y, 0])
                                spma.PointManip.Transform.translate(item['collider2'].get_path(), [0, delta_y, 0])
                                bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(
                                    item['collider1'].get_path() +
                                    item['collider2'].get_path())
                                last_pos_y += delta_y
                                item['y_pos'] += delta_y
                                y_set = True
                        (left, bottom, right, top) = bb_curr[0]['x'], bb_curr[0]['y'], bb_curr[1]['x'], bb_curr[1][
                            'y']
                        intersections = list(
                            self.r_index[self.num_sections - 1].intersection((left, bottom, right, top)))
                        if len(intersections) == 0:
                            item['cut'] = self.num_sections
                            self.r_index[self.num_sections - 1].insert(id=r_id, coordinates=(left, bottom, right, top))
                            r_id += 1
                            placed = True
                    else:
                        delta_x = -left
                        spma.PointManip.Transform.translate(item['collider1'].get_path(), [delta_x, 0, 0])
                        spma.PointManip.Transform.translate(item['collider2'].get_path(), [delta_x, 0, 0])
                        bb_curr = prim.GeometricFunctions.get_bounding_box_from_path(item['collider1'].get_path() +
                                                                                     item['collider2'].get_path())
                        (left, bottom, right, top) = bb_curr[0]['x'], bb_curr[0]['y'], bb_curr[1]['x'], bb_curr[1]['y']
                        last_pos_y += delta_x
                        item['y_pos'] += delta_x
                if iterations > time_out:
                    iterations = 0
                    last_pos_x = 0
                    last_pos_y = self.work_piece.height
                    self.num_sections += 1
                    self.r_index.append(rtree.index.Index())
                else:
                    iterations += 1
            prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
                                                                     index='%s' % indx, num_sections=self.num_sections)
            indx += 1
        prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
                                                                 index='final', num_sections=self.num_sections)
        self.apply_state_dict()

    def apply_state_dict(self):
        for entry in self.state_dict.values():
            center = self.wire_cutter.wire_length / 2
            translate1 = [entry['x_pos'], entry['y_pos'], center - self.work_piece.thickness]
            translate2 = [entry['x_pos'], entry['y_pos'], center + self.work_piece.thickness]
            sec_1_path = entry['section'].section1.get_path()
            sec_2_path = entry['section'].section2.get_path()
            spma.PointManip.Transform.rotate(sec_1_path, [0, 0, np.deg2rad(entry['rot'])])
            spma.PointManip.Transform.rotate(sec_2_path, [0, 0, np.deg2rad(entry['rot'])])
            spma.PointManip.Transform.translate(sec_1_path, translate1)
            spma.PointManip.Transform.translate(sec_2_path, translate2)
            entry['bb'] = prim.GeometricFunctions.get_bounding_box_from_path(entry['collider1'].get_path() +
                                                                             entry['collider2'].get_path())
            assert len(sec_1_path) > 1, 'Error: Section 1 of State dict entry [%s] has no Points.' % entry
            assert len(sec_2_path) > 1, 'Error: Section 2 of State dict entry [%s] has no Points.' % entry

    def calculate_initial_rotation(self):
        for ind in self.state_dict:
            path = self.state_dict[ind]['collider1'].get_path()
            path2 = self.state_dict[ind]['collider2'].get_path()
            furthest_dist = 0
            p1 = path[0]
            p2 = path[1]
            for point1 in range(0, len(path) - 1):
                for point2 in range(point1 + 1, len(path)):
                    dist = np.sqrt((path[point2] - path[point1]) ** 2)
                    if dist > furthest_dist:
                        p1 = path[point1]
                        p2 = path[point2]
                        furthest_dist = dist
            angle = np.arctan2(p1['y'] - p2['y'], p1['x'] - p2['x'])
            self.state_dict[ind]['rot'] = np.rad2deg(-angle) + 90
            spma.PointManip.Transform.rotate(path, [0, 0, -angle + np.pi / 2])
            spma.PointManip.Transform.rotate(path2, [0, 0, -angle + np.pi / 2])

    def create_section_links_for_cross_section_pairs(self, method=0):
        """

        :return:
        :rtype: list[list[prim.SectionLink or comp.CrossSection]]
        """
        if method == 0:
            cut_list_1, cut_list_2 = self.create_section_links_for_cross_section_pairs_horz()
        elif method == 1:
            cut_list_1, cut_list_2 = self.create_section_links_for_cross_section_pairs_vert()
        return cut_list_1, cut_list_2

    def create_section_links_for_cross_section_pairs_horz(self):
        logger = logging.getLogger(__name__)
        cut_list_1 = list()
        cut_list_2 = list()

        z_1 = self.state_dict[0]['section'].section1.get_path()[0]['z']
        z_2 = self.state_dict[0]['section'].section2.get_path()[0]['z']
        for ind in range(self.num_sections):
            cut_list_1.append(list())
            cut_list_2.append(list())
            section_order_list = self._get_section_order(ind)
            prev_point_1 = prim.Point(0, 0, z_1)
            prev_point_2 = prim.Point(0, 0, z_2)
            for y_axis in section_order_list:
                min_ind = prim.GeometricFunctions.get_index_min_coord(
                    self.state_dict[y_axis[0]]['section'].section1.get_path(), 'x')
                y_start = self.state_dict[y_axis[0]]['section'].section1.get_path()[min_ind]['y']
                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1), fast_cut=True))
                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2), fast_cut=True))
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

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1[min_ind]['x'],
                                                                                     section_1[min_ind]['y'], z_1)))
                    logger.debug('ind: %s, len: %s min_ind: %s size section2: %s', ind, len(cut_list_2), min_ind,
                                 len(section_2))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2[min_ind]['x'],
                                                                                     section_2[min_ind]['y'], z_2)))

                    prev_point_1, prev_point_2 = SpatialPlacement.add_section(cut_list_1[ind], cut_list_2[ind], False,
                                                                              section_1,
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
                        cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1[max_ind]['x'],
                                                                                         section_1[max_ind]['y'], z_1)))
                        cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2[max_ind]['x'],
                                                                                         section_2[max_ind]['y'], z_2)))

                    prev_point_1, prev_point_2 = SpatialPlacement.add_section(cut_list_1[ind], cut_list_2[ind], True,
                                                                              section_1,
                                                                              section_2, z_1, z_2)

                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1)))
                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2)))

                prev_point_1 = prim.Point(0, y_start, z_1)
                prev_point_2 = prim.Point(0, y_start, z_2)

        return cut_list_1, cut_list_2

    def create_section_links_for_cross_section_pairs_vert(self):
        logger = logging.getLogger(__name__)
        cut_list_1 = list()
        cut_list_2 = list()

        z_1 = self.state_dict[0]['section'].section1.get_path()[0]['z']
        z_2 = self.state_dict[0]['section'].section2.get_path()[0]['z']
        for ind in range(self.num_sections):
            cut_list_1.append(list())
            cut_list_2.append(list())
            section_order_list = self._get_section_order(ind)
            prev_point_1 = prim.Point(0, 0, z_1)
            prev_point_2 = prim.Point(0, 0, z_2)
            for y_axis in section_order_list:
                max_ind = prim.GeometricFunctions.get_index_max_coord(
                    self.state_dict[y_axis[0]]['section'].section1.get_path(), 'y')
                y_start = self.state_dict[y_axis[0]]['section'].section1.get_path()[max_ind]['y']
                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1), fast_cut=True))
                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2), fast_cut=True))
                prev_point_1 = prim.Point(0, y_start, z_1)
                prev_point_2 = prim.Point(0, y_start, z_2)
                for x_axis in y_axis:
                    section = self.state_dict[x_axis]['section']
                    section_1 = section.section1.get_path()
                    section_2 = section.section2.get_path()

                    err_prompt = 'Error Sections do not have the same number of points S1 %s, S2 %s' % \
                                 (len(section_1), len(section_2))
                    assert abs(len(section_1) - len(section_2)) <= 1, err_prompt

                    max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, 'y')

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1[max_ind]['x'],
                                                                                     section_1[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2[max_ind]['x'],
                                                                                     section_2[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_2)))

                    prev_point_1 = prim.Point(section_1[max_ind]['x'], section_1[max_ind]['y'] + self.vert_spacing, z_1)
                    prev_point_2 = prim.Point(section_2[max_ind]['x'], section_2[max_ind]['y'] + self.vert_spacing, z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1[max_ind]['x'],
                                                                                     section_1[max_ind]['y'], z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2[max_ind]['x'],
                                                                                     section_2[max_ind]['y'], z_2)))

                    prev_point_1, prev_point_2 = SpatialPlacement.add_section_vert(cut_list_1[ind], cut_list_2[ind],
                                                                                   section_1, section_2, z_1, z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1[max_ind]['x'],
                                                                                     section_1[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2[max_ind]['x'],
                                                                                     section_2[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_2)))

                    prev_point_1 = prim.Point(section_1[max_ind]['x'], section_1[max_ind]['y'] + self.vert_spacing, z_1)
                    prev_point_2 = prim.Point(section_2[max_ind]['x'], section_2[max_ind]['y'] + self.vert_spacing, z_2)

                for x_axis in reversed(y_axis):
                    section = self.state_dict[x_axis]['section']
                    section_1 = section.section1.get_path()
                    section_2 = section.section2.get_path()

                    max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, 'y')
                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1[max_ind]['x'],
                                                                                     section_1[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2[max_ind]['x'],
                                                                                     section_2[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_2)))

                    prev_point_1 = prim.Point(section_1[max_ind]['x'], section_1[max_ind]['y'] + self.vert_spacing, z_1)
                    prev_point_2 = prim.Point(section_2[max_ind]['x'], section_2[max_ind]['y'] + self.vert_spacing, z_2)


                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1)))
                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2)))

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
                cut_list_1.append(comp.CrossSection(prim.Path(section_1[max_ind:min_ind + 1])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[max_ind:min_ind + 1])))
                point1 = section_1[min_ind]
                point2 = section_2[min_ind]
            else:
                path1 = prim.Path(section_1[min_ind:] + section_1[0:max_ind + 1])
                path2 = prim.Path(section_2[min_ind:] + section_2[0:max_ind + 1])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))
                point1 = section_1[max_ind]
                point2 = section_2[max_ind]

        else:
            if reverse_dir:
                path1 = prim.Path(section_1[max_ind:] + section_1[0:min_ind + 1])
                path2 = prim.Path(section_2[max_ind:] + section_2[0:min_ind + 1])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))
                point1 = section_1[min_ind]
                point2 = section_2[min_ind]

            else:
                cut_list_1.append(comp.CrossSection(prim.Path(section_1[min_ind:max_ind + 1])))
                cut_list_2.append(comp.CrossSection(prim.Path(section_2[min_ind:max_ind + 1])))
                point1 = section_1[max_ind]
                point2 = section_2[max_ind]

        if point1:
            point1['z'] = z_1
            point2['z'] = z_2
        return point1, point2

    @staticmethod
    def add_section_vert(cut_list_1, cut_list_2, section_1, section_2, z_1, z_2):
        max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, 'y')
        path1 = prim.Path(section_1[max_ind:] + section_1[0:max_ind + 1])
        path2 = prim.Path(section_2[max_ind:] + section_2[0:max_ind + 1])
        cut_list_1.append(comp.CrossSection(path1))
        cut_list_2.append(comp.CrossSection(path2))
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
        :rtype: (comp.CrossSection, comp.CrossSection)
        """
        path1 = prim.GeometricFunctions.offset_curve(section.section1.get_path(), offset_scale=wall_thickness, dir=0,
                                                     divisions=1, add_leading_edge=False)

        path1 = prim.GeometricFunctions.normalize_path_points(path1, num_points=32)
        path1 = prim.GeometricFunctions.close_path(path1)

        path2 = prim.GeometricFunctions.offset_curve(section.section2.get_path(), offset_scale=wall_thickness, dir=0,
                                                     divisions=1, add_leading_edge=False)

        path2 = prim.GeometricFunctions.normalize_path_points(path2, num_points=32)
        path2 = prim.GeometricFunctions.close_path(path2)

        return comp.CrossSection(prim.Path(path1)), comp.CrossSection(prim.Path(path2))

    def _get_section_order(self, cut_piece):
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
            if self.state_dict[ind]['cut'] == cut_piece + 1:
                if self.state_dict[ind]['bb'][0]['x'] <= 0:
                    y_pos = (self.state_dict[ind]['bb'][1]['y'] + self.state_dict[ind]['bb'][0]['y']) / 2
                    x_pos = (self.state_dict[ind]['bb'][1]['x'] + self.state_dict[ind]['bb'][0]['x']) / 2
                    section_order_dict[y_pos] = dict()
                    section_order_dict[y_pos][x_pos] = ind
                    ind_taken.append(ind)

        for ind in self.state_dict:
            if self.state_dict[ind]['cut'] == cut_piece + 1:
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
                if self.state_dict[section_order_dict[ind_y][ind_x]]['cut'] == cut_piece + 1:
                    tmp_lst.append(section_order_dict[ind_y][ind_x])
            section_order_list.append(tmp_lst)

        self.section_order.append(section_order_list)
        return self.section_order[cut_piece]

    def plot_section_order(self, output_dir):
        for cut_section in range(self.num_sections):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=320)
            for ind1 in range(0, len(self.section_order[cut_section])):
                for ind2 in range(0, len(self.section_order[cut_section][ind1])):
                    ind = self.section_order[cut_section][ind1][ind2]
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
        for cut_section in range(self.num_sections):
            section_order_list = self.section_order[cut_section]
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
                        prim.GeometricFunctions.plot_path(section_1[min_ind:max_ind + 1], color='C1', scatter=False)
                        prim.GeometricFunctions.plot_path(section_1[max_ind:], color='C2', scatter=False)
                        prim.GeometricFunctions.plot_path(section_1[0:min_ind + 1], color='C2', scatter=False)

                        prim.GeometricFunctions.plot_path(section_2[min_ind:max_ind + 1], color='C3', scatter=False)
                        prim.GeometricFunctions.plot_path(section_2[max_ind:], color='C4', scatter=False)
                        prim.GeometricFunctions.plot_path(section_2[0:min_ind + 1], color='C4', scatter=False)
                    else:
                        prim.GeometricFunctions.plot_path(section_1[min_ind:], color='C5', scatter=False)
                        prim.GeometricFunctions.plot_path(section_1[0:max_ind + 1], color='C5', scatter=False)
                        prim.GeometricFunctions.plot_path(section_1[max_ind:min_ind + 1], color='C6', scatter=False)

                        prim.GeometricFunctions.plot_path(section_2[min_ind:], color='C7', scatter=False)
                        prim.GeometricFunctions.plot_path(section_2[0:max_ind + 1], color='C7', scatter=False)
                        prim.GeometricFunctions.plot_path(section_2[max_ind:min_ind + 1], color='C8', scatter=False)
                    plt.axis('equal')
                    plt.savefig(os.path.join(output_dir, 'Cross_Section_cut_splitting_debug_%s.png' % ind))
                    ind += 1
