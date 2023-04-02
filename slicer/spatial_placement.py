import logging
import os
import sys

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

    NO_OVERLAP = 0
    ABOVE = 1
    BELOW = 2

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

            center = prim.Point(0, 0, 0)
            for point in path_1:
                center += point
            for point in path_2:
                center += point
            center /= (len(path_1) + len(path_2))
            spma.PointManip.Transform.translate(path_1, -center)
            spma.PointManip.Transform.translate(path_2, -center)
            if section.section1.get_path_hole() is not None:
                for hole in section.section1.get_path_hole():
                    spma.PointManip.Transform.translate(hole, -center)
            if section.section2.get_path_hole() is not None:
                for hole in section.section2.get_path_hole():
                    spma.PointManip.Transform.translate(hole, -center)

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

        self.calculate_initial_rotation()

        r_id = 0

        last_pos_x = 0
        last_pos_y = self.work_piece.height
        indx = 0
        time_out = 10000

        for ind in reversed(ordered_state_indexes):
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
            # prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
            #                                                          index='%s' % indx, num_sections=self.num_sections)
            indx += 1
        for ind in range(self.num_sections):
            prim.GeometricFunctions.plot_cross_sections_on_workpiece(self.state_dict, self.work_piece, output_dir,
                                                                     index='final', num_sections=ind + 1)
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

            if entry['section'].section1.get_path_hole() is not None:
                for hole in entry['section'].section1.get_path_hole():
                    sec_1_path_hole = hole
                    spma.PointManip.Transform.rotate(sec_1_path_hole, [0, 0, np.deg2rad(entry['rot'])])
                    spma.PointManip.Transform.translate(sec_1_path_hole, translate1)

            if entry['section'].section2.get_path_hole() is not None:
                for hole in entry['section'].section2.get_path_hole():
                    sec_2_path_hole = hole
                    spma.PointManip.Transform.rotate(sec_2_path_hole, [0, 0, np.deg2rad(entry['rot'])])
                    spma.PointManip.Transform.translate(sec_2_path_hole, translate2)

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

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1,
                                                            prim.Point(section_1[min_ind]['x'], section_1[min_ind]['y'],
                                                                       z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2,
                                                            prim.Point(section_2[min_ind]['x'], section_2[min_ind]['y'],
                                                                       z_2)))

                    prev_point_1, prev_point_2 = SpatialPlacement.add_section(cut_list_1[ind], cut_list_2[ind], False,
                                                                              section_1, section_2, z_1, z_2)

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

            overlapped_items = self._get_overlap_items(ind)
            overlapped_items = self._clean_overlap_dict(overlapped_items, ind)
            for entry in overlapped_items.values():
                for item in entry['above']:
                    for y_axis in section_order_list:
                        if item in y_axis:
                            y_axis.remove(item)
                for item in entry['below']:
                    for y_axis in section_order_list:
                        if item in y_axis:
                            y_axis.remove(item)

            prev_point_1 = prim.Point(0, 0, z_1)
            prev_point_2 = prim.Point(0, 0, z_2)
            for y_axis in section_order_list:
                max_ind = prim.GeometricFunctions.get_index_max_coord(
                    self.state_dict[y_axis[0]]['section'].section1.get_path(), 'y')
                y_start = max(self.state_dict[y_axis[0]]['section'].section1.get_path()[max_ind]['y'],
                              self.state_dict[y_axis[0]]['section'].section2.get_path()[max_ind]['y'])
                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1), fast_cut=True))
                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2), fast_cut=True))
                prev_point_1 = prim.Point(0, y_start, z_1)
                prev_point_2 = prim.Point(0, y_start, z_2)
                for index, x_axis in enumerate(y_axis):
                    section = self.state_dict[x_axis]['section']
                    section_1_path = section.section1.get_path()
                    section_2_path = section.section2.get_path()
                    bb = self.state_dict[x_axis]['bb']
                    bb_next = None
                    if index + 1 < len(y_axis):
                        bb_next = self.state_dict[y_axis[index + 1]]['bb']

                    err_prompt = 'Error Sections do not have the same number of points S1 %s, S2 %s' % \
                                 (len(section_1_path), len(section_2_path))
                    assert abs(len(section_1_path) - len(section_2_path)) <= 1, err_prompt

                    max_ind = prim.GeometricFunctions.get_index_max_coord(section_1_path, 'y')

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb[0][0], bb[1][1], z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb[0][0], bb[1][1], z_2)))

                    prev_point_1 = prim.Point(bb[0][0], bb[1][1], z_1)
                    prev_point_2 = prim.Point(bb[0][0], bb[1][1], z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1_path[max_ind]['x'],
                                                                                     section_1_path[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2_path[max_ind]['x'],
                                                                                     section_2_path[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_2)))

                    prev_point_1 = prim.Point(section_1_path[max_ind]['x'],
                                              section_1_path[max_ind]['y'] + self.vert_spacing, z_1)
                    prev_point_2 = prim.Point(section_2_path[max_ind]['x'],
                                              section_2_path[max_ind]['y'] + self.vert_spacing, z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1,
                                                            prim.Point(section_1_path[max_ind]['x'],
                                                                       section_1_path[max_ind]['y'],
                                                                       z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2,
                                                            prim.Point(section_2_path[max_ind]['x'],
                                                                       section_2_path[max_ind]['y'],
                                                                       z_2)))

                    if x_axis in overlapped_items:
                        prev_point_1, prev_point_2 = self.add_section_vert_w_overlap(cut_list_1[ind], cut_list_2[ind],
                                                                                     z_1, z_2, x_axis, overlapped_items)
                    else:
                        prev_point_1, prev_point_2 = SpatialPlacement.add_section_vert(cut_list_1[ind], cut_list_2[ind],
                                                                                       section.section1,
                                                                                       section.section2, z_1, z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1_path[max_ind]['x'],
                                                                                     section_1_path[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2_path[max_ind]['x'],
                                                                                     section_2_path[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_2)))

                    prev_point_1 = prim.Point(section_1_path[max_ind]['x'],
                                              section_1_path[max_ind]['y'] + self.vert_spacing, z_1)
                    prev_point_2 = prim.Point(section_2_path[max_ind]['x'],
                                              section_2_path[max_ind]['y'] + self.vert_spacing, z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb[1][0], bb[1][1], z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb[1][0], bb[1][1], z_2)))

                    prev_point_1 = prim.Point(bb[1][0], bb[1][1], z_1)
                    prev_point_2 = prim.Point(bb[1][0], bb[1][1], z_2)

                    if bb_next is not None:
                        p1 = prim.Point(bb[1][0], bb[1][1], z_1)
                        p2 = prim.Point(bb_next[0][0], bb[1][1], z_1)
                        intersections = self.find_intersections_between_two_points(p1, p2, ind + 1, x_axis,
                                                                                   y_axis[index + 1])
                        if len(intersections) > 0:
                            intersect_dict = dict()
                            for intersection in intersections:
                                intersect_dict[self.state_dict[intersection]['x_pos']] = intersection
                            for intersection in sorted(intersect_dict):
                                bb_curr = self.state_dict[intersect_dict[intersection]]['bb']
                                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb_curr[0][0],
                                                                                                 bb_curr[0][1], z_1)))
                                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb_curr[0][0],
                                                                                                 bb_curr[0][1], z_2)))

                                prev_point_1 = prim.Point(bb_curr[0][0], bb_curr[0][1], z_1)
                                prev_point_2 = prim.Point(bb_curr[0][0], bb_curr[0][1], z_2)

                                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb_curr[1][0],
                                                                                                 bb_curr[0][1], z_1)))
                                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb_curr[1][0],
                                                                                                 bb_curr[0][1], z_2)))

                                prev_point_1 = prim.Point(bb_curr[1][0], bb_curr[0][1], z_1)
                                prev_point_2 = prim.Point(bb_curr[1][0], bb_curr[0][1], z_2)

                        cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb_next[0][0],
                                                                                         prev_point_1['y'], z_1)))
                        cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb_next[0][0],
                                                                                         prev_point_2['y'], z_2)))

                        prev_point_1 = prim.Point(bb_next[0][0], prev_point_1['y'], z_1)
                        prev_point_2 = prim.Point(bb_next[0][0], prev_point_2['y'], z_2)

                        cut_list_1[ind].append(
                            prim.SectionLink(prev_point_1, prim.Point(bb_next[0][0], bb_next[1][1], z_1)))
                        cut_list_2[ind].append(
                            prim.SectionLink(prev_point_2, prim.Point(bb_next[0][0], bb_next[1][1], z_2)))

                        prev_point_1 = prim.Point(bb_next[0][0], bb_next[1][1], z_1)
                        prev_point_2 = prim.Point(bb_next[0][0], bb_next[1][1], z_2)

                for index, x_axis in enumerate(reversed(y_axis)):
                    section = self.state_dict[x_axis]['section']
                    section_1_path = section.section1.get_path()
                    section_2_path = section.section2.get_path()
                    bb = self.state_dict[x_axis]['bb']
                    bb_next = None
                    if (len(y_axis) - 1 - index) - 1 >= 0:
                        bb_next = self.state_dict[y_axis[(len(y_axis) - 1 - index) - 1]]['bb']

                    max_ind = prim.GeometricFunctions.get_index_max_coord(section_1_path, 'y')

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb[1][0], bb[1][1], z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb[1][0], bb[1][1], z_2)))

                    prev_point_1 = prim.Point(bb[1][0], bb[1][1], z_1)
                    prev_point_2 = prim.Point(bb[1][0], bb[1][1], z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(section_1_path[max_ind]['x'],
                                                                                     section_1_path[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(section_2_path[max_ind]['x'],
                                                                                     section_2_path[max_ind][
                                                                                         'y'] + self.vert_spacing,
                                                                                     z_2)))

                    prev_point_1 = prim.Point(section_1_path[max_ind]['x'],
                                              section_1_path[max_ind]['y'] + self.vert_spacing, z_1)
                    prev_point_2 = prim.Point(section_2_path[max_ind]['x'],
                                              section_2_path[max_ind]['y'] + self.vert_spacing, z_2)

                    cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb[0][0], bb[1][1], z_1)))
                    cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb[0][0], bb[1][1], z_2)))

                    prev_point_1 = prim.Point(bb[0][0], bb[1][1], z_1)
                    prev_point_2 = prim.Point(bb[0][0], bb[1][1], z_2)

                    if bb_next is not None:
                        # cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb[0][0], bb_next[1][1], z_1)))
                        # cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb[0][0], bb_next[1][1], z_2)))
                        #
                        # prev_point_1 = prim.Point(bb[0][0], bb_next[1][1], z_1)
                        # prev_point_2 = prim.Point(bb[0][0], bb_next[1][1], z_2)

                        p1 = prim.Point(bb[0][0], bb_next[1][1], z_1)
                        p2 = prim.Point(bb_next[1][0], bb_next[1][1], z_1)
                        intersections = self.find_intersections_between_two_points(p1, p2, ind + 1, x_axis, y_axis[
                            (len(y_axis) - 1 - index) - 1])
                        if len(intersections) > 0:
                            intersect_dict = dict()
                            for intersection in intersections:
                                intersect_dict[self.state_dict[intersection]['x_pos']] = intersection
                            for intersection in sorted(intersect_dict.keys(), reverse=True):
                                bb_curr = self.state_dict[intersect_dict[intersection]]['bb']

                                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(prev_point_1['x'],
                                                                                                 bb_curr[0][1], z_1)))
                                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(prev_point_1['x'],
                                                                                                 bb_curr[0][1], z_2)))

                                prev_point_1 = prim.Point(prev_point_1['x'], bb_curr[0][1], z_1)
                                prev_point_2 = prim.Point(prev_point_1['x'], bb_curr[0][1], z_2)

                                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb_curr[1][0],
                                                                                                 bb_curr[0][1], z_1)))
                                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb_curr[1][0],
                                                                                                 bb_curr[0][1], z_2)))

                                prev_point_1 = prim.Point(bb_curr[1][0], bb_curr[0][1], z_1)
                                prev_point_2 = prim.Point(bb_curr[1][0], bb_curr[0][1], z_2)

                                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(bb_curr[0][0],
                                                                                                 bb_curr[0][1], z_1)))
                                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(bb_curr[0][0],
                                                                                                 bb_curr[0][1], z_2)))

                                prev_point_1 = prim.Point(bb_curr[0][0], bb_curr[0][1], z_1)
                                prev_point_2 = prim.Point(bb_curr[0][0], bb_curr[0][1], z_2)

                        cut_list_1[ind].append(
                            prim.SectionLink(prev_point_1, prim.Point(prev_point_1['x'], bb_next[1][1], z_1)))
                        cut_list_2[ind].append(
                            prim.SectionLink(prev_point_2, prim.Point(prev_point_2['x'], bb_next[1][1], z_2)))

                        prev_point_1 = prim.Point(prev_point_1['x'], bb_next[1][1], z_1)
                        prev_point_2 = prim.Point(prev_point_2['x'], bb_next[1][1], z_2)

                        cut_list_1[ind].append(
                            prim.SectionLink(prev_point_1, prim.Point(bb_next[1][0], bb_next[1][1], z_1)))
                        cut_list_2[ind].append(
                            prim.SectionLink(prev_point_2, prim.Point(bb_next[1][0], bb_next[1][1], z_2)))

                        prev_point_1 = prim.Point(bb_next[1][0], bb_next[1][1], z_1)
                        prev_point_2 = prim.Point(bb_next[1][0], bb_next[1][1], z_2)

                cut_list_1[ind].append(prim.SectionLink(prev_point_1, prim.Point(0, y_start, z_1)))
                cut_list_2[ind].append(prim.SectionLink(prev_point_2, prim.Point(0, y_start, z_2)))

                prev_point_1 = prim.Point(0, y_start, z_1)
                prev_point_2 = prim.Point(0, y_start, z_2)

        return cut_list_1, cut_list_2

    def find_intersections_between_two_points(self, point1, point2, curr_cut, ind1, ind2):
        """
        Returns a list of indexes in the state_dict that have bounding boxes intersecting the line formed by point1 to
        point2.

        :param point1:
        :param point2:
        :param curr_cut:
        :param ind1:
        :param ind2:
        :return:
        """

        line = prim.Line.line_from_points(point1, point2)
        intersect_list = list()
        for ind in self.state_dict:
            if curr_cut == self.state_dict[ind]['cut'] and ind != ind1 and ind != ind2:
                bb = self.state_dict[ind]['bb']
                point_min_y = prim.Point(bb[0][0], bb[0][1], point1['z'])
                point_max_y = prim.Point(bb[0][0], bb[1][1], point1['z'])
                vert_line = prim.Line.line_from_points(point_min_y, point_max_y)
                intersects, _ = vert_line.intersects(line)
                if intersects:
                    y_avg = sum([point1['y'], point2['y']]) / 2.0
                    if abs(y_avg - point_max_y['y']) > 15.0 or abs(y_avg - point_min_y['y']) > 10.0:
                        intersect_list.append(ind)
        return intersect_list

    @staticmethod
    def add_section(cut_list_1, cut_list_2, reverse_dir, section_1, section_2, z_1, z_2, dimension='x'):
        min_ind = prim.GeometricFunctions.get_index_min_coord(section_1, dimension)
        max_ind = prim.GeometricFunctions.get_index_max_coord(section_1, dimension)
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
    def add_section_vert(cut_list_1, cut_list_2, section_1, section_2, z_1, z_2, bottom=False):
        """

        :param cut_list_1:
        :param cut_list_2:
        :param section_1:
        :param section_2:
        :param z_1:
        :param z_2:
        :param bottom:
        :return:
        """
        logger = logging.getLogger(__name__)

        section_path1 = section_1.get_path()
        section_path2 = section_2.get_path()
        ind = prim.GeometricFunctions.get_index_min_coord(section_path1, 'y') if bottom else \
            prim.GeometricFunctions.get_index_max_coord(section_path1, 'y')

        if section_1.get_path_hole() is not None or section_2.get_path_hole() is not None:
            ind_list_holes = section_1.get_closest_point_to_hole() if section_1.get_path_hole() is not None else \
                section_2.get_closest_point_to_hole()
            holes1 = section_1.get_path_hole()
            holes2 = section_2.get_path_hole()
            center1 = prim.GeometricFunctions.get_center_of_path(section_path1)
            center2 = prim.GeometricFunctions.get_center_of_path(section_path2)
            logger.debug(ind_list_holes)
            hole_order, inds_prim, inds_hole = SpatialPlacement.get_hole_index_order(holes1,
                                                                                     ind_list_holes) if holes1 is not None else SpatialPlacement.get_hole_index_order(
                holes2, ind_list_holes)

            # If the first hole occurs after the starting cut index, then setup the cutpath past the first hole, then
            # look through the remaining holes.
            start_ind = -1
            ind_in_list = -1
            for num, index in enumerate(inds_prim):
                if index > ind:
                    start_ind = index
                    ind_in_list = num
                    break
            if start_ind != -1:
                # There is a at least one hole that occurs after the start index of the cross section cut.
                ind2 = inds_prim[ind_in_list]
                hole_ind = inds_hole[ind_in_list]

                hole1 = holes1[hole_order[ind_in_list]] if holes1 is not None else None
                hole2 = holes2[hole_order[ind_in_list]] if holes2 is not None else None

                path1 = prim.Path(section_path1[ind:ind2+1])
                path2 = prim.Path(section_path2[ind:ind2+1])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))

                section_link1 = prim.SectionLink(section_path1[ind2], hole1[hole_ind]) if hole1 is not None \
                    else prim.SectionLink(section_path1[ind2], center1)
                section_link2 = prim.SectionLink(section_path2[ind2], hole2[hole_ind]) if hole2 is not None \
                    else prim.SectionLink(section_path2[ind2], center2)
                cut_list_1.append(section_link1)
                cut_list_2.append(section_link2)

                path1 = prim.Path(hole1[hole_ind:] + hole1[0:hole_ind + 1]) if hole1 is not None else prim.Path([center1] * len(hole2))
                path2 = prim.Path(hole2[hole_ind:] + hole2[0:hole_ind + 1]) if hole2 is not None else prim.Path([center2] * len(hole1))
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))

                section_link1 = prim.SectionLink(hole1[hole_ind], section_path1[ind2]) if hole1 is not None \
                    else prim.SectionLink(center1, section_path1[ind2])
                section_link2 = prim.SectionLink(hole2[hole_ind], section_path2[ind2]) if hole2 is not None \
                    else prim.SectionLink(center2, section_path2[ind2])
                cut_list_1.append(section_link1)
                cut_list_2.append(section_link2)

                # Loop through the holes and create the cutpaths for them. If there is only 1 hole then the loop will
                # not execute.
                for ind_next in range(ind_in_list + 1, len(hole_order)):
                    prev_ind = inds_prim[ind_next - 1]
                    ind2 = inds_prim[ind_next]
                    hole_ind = inds_hole[ind_next]
                    hole1 = holes1[hole_order[ind_next]]
                    hole2 = holes2[hole_order[ind_next]]

                    path1 = prim.Path(section_path1[prev_ind:ind2+1])
                    path2 = prim.Path(section_path2[prev_ind:ind2+1])
                    cut_list_1.append(comp.CrossSection(path1))
                    cut_list_2.append(comp.CrossSection(path2))

                    section_link1 = prim.SectionLink(section_path1[ind2], hole1[hole_ind]) if hole1 is not None \
                        else prim.SectionLink(section_path1[ind2], center1)
                    section_link2 = prim.SectionLink(section_path2[ind2], hole2[hole_ind]) if hole2 is not None \
                        else prim.SectionLink(section_path2[ind2], center2)
                    cut_list_1.append(section_link1)
                    cut_list_2.append(section_link2)

                    path1 = prim.Path(hole1[hole_ind:] + hole1[0:hole_ind + 1]) if hole1 is not None else prim.Path([center1] * len(hole2))
                    path2 = prim.Path(hole2[hole_ind:] + hole2[0:hole_ind + 1]) if hole2 is not None else prim.Path([center2] * len(hole1))
                    cut_list_1.append(comp.CrossSection(path1))
                    cut_list_2.append(comp.CrossSection(path2))

                    section_link1 = prim.SectionLink(hole1[hole_ind], section_path1[ind2]) if hole1 is not None \
                        else prim.SectionLink(center1, section_path1[ind2])
                    section_link2 = prim.SectionLink(hole2[hole_ind], section_path2[ind2]) if hole2 is not None \
                        else prim.SectionLink(center2, section_path2[ind2])
                    cut_list_1.append(section_link1)
                    cut_list_2.append(section_link2)

                # Complete the circuit from the last hole position to the final point in the cross section path
                path1 = prim.Path(section_path1[inds_prim[-1]:])
                path2 = prim.Path(section_path2[inds_prim[-1]:])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))

                # todo: check for any holes prior to ind before adding path from 0:ind+1
                # All holes occur before the start index of the cut.
                # Need to include the final point from the previous segment to prevent discontinuities
                path1 = prim.Path([section_path1[-1]] + section_path1[0:ind+1])
                path2 = prim.Path([section_path2[-1]] + section_path2[0:ind+1])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))
            else:
                # All holes occur before the start index of the cut.
                path1 = prim.Path(section_path1[ind:])
                path2 = prim.Path(section_path2[ind:])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))

                path1 = prim.Path(section_path1[0:inds_prim[0]+1])
                path2 = prim.Path(section_path2[0:inds_prim[0]+1])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))

                # Loop through the holes and create the cutpaths for them. If there is only 1 hole then the loop will
                # not execute.
                for ind_next in range(start_ind + 1, len(hole_order)):
                    ind2 = inds_prim[ind_next]
                    hole_ind = inds_hole[ind_next]
                    hole1 = holes1[hole_order[ind_next]]
                    hole2 = holes2[hole_order[ind_next]]

                    section_link1 = prim.SectionLink(section_path1[ind2], hole1[hole_ind]) if hole1 is not None \
                        else prim.SectionLink(section_path1[ind2], center1)
                    section_link2 = prim.SectionLink(section_path2[ind2], hole2[hole_ind]) if hole2 is not None \
                        else prim.SectionLink(section_path2[ind2], center2)
                    cut_list_1.append(section_link1)
                    cut_list_2.append(section_link2)

                    path1 = prim.Path(hole1[hole_ind:] + hole1[0:hole_ind + 1]) if hole1 is not None else prim.Path([center1] * len(hole2))
                    path2 = prim.Path(hole2[hole_ind:] + hole2[0:hole_ind + 1]) if hole2 is not None else prim.Path([center2] * len(hole1))
                    cut_list_1.append(comp.CrossSection(path1))
                    cut_list_2.append(comp.CrossSection(path2))

                    section_link1 = prim.SectionLink(hole1[hole_ind], section_path1[ind2]) if hole1 is not None \
                        else prim.SectionLink(center1, section_path1[ind2])
                    section_link2 = prim.SectionLink(hole2[hole_ind], section_path2[ind2]) if hole2 is not None \
                        else prim.SectionLink(center2, section_path2[ind2])
                    cut_list_1.append(section_link1)
                    cut_list_2.append(section_link2)

                    # todo: This case is not complete. Will most likely only work for a single hole

                path1 = prim.Path(section_path1[inds_prim[-1]:ind+1])
                path2 = prim.Path(section_path2[inds_prim[-1]:ind+1])
                cut_list_1.append(comp.CrossSection(path1))
                cut_list_2.append(comp.CrossSection(path2))

        else:
            path1 = prim.Path(section_path1[ind:] + section_path1[0:ind + 1])
            path2 = prim.Path(section_path2[ind:] + section_path2[0:ind + 1])
            cut_list_1.append(comp.CrossSection(path1))
            cut_list_2.append(comp.CrossSection(path2))

        point1 = section_path1[ind]
        point2 = section_path2[ind]

        if point1:
            point1['z'] = z_1
            point2['z'] = z_2
        return point1, point2

    def add_section_vert_w_overlap(self, cut_list_1, cut_list_2, z_1, z_2, entry, overlap):
        """
        Handles Creating cutpaths for CrossSections that have other vertical overlapping CrossSections.

        :param cut_list_1:
        :param cut_list_2:
        :param z_1:
        :param z_2:
        :param entry:
        :param overlap:
        :return:
        """
        main_section_1 = self.state_dict[entry]['section'].section1.get_path()
        main_section_2 = self.state_dict[entry]['section'].section2.get_path()
        min_ind = prim.GeometricFunctions.get_index_min_coord(main_section_1, 'y')

        # Add the cutpath going from the top to the bottom of the primary section for the overlap
        prev_point_1, prev_point_2 = SpatialPlacement.add_section(cut_list_1, cut_list_2, True, main_section_1,
                                                                  main_section_2, z_1, z_2, dimension='y')

        for item in overlap[entry]['below']:
            section_tmp = self.state_dict[item]['section']
            section_tmp1_path = section_tmp.section1.get_path()
            section_tmp2_path = section_tmp.section2.get_path()
            max_ind = prim.GeometricFunctions.get_index_max_coord(section_tmp1_path, 'y')

            # Move from the bottom of the primary CrossSection to the next overlapped CrossSection + Vert spacing
            cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(section_tmp1_path[max_ind]['x'],
                                                                        section_tmp1_path[max_ind][
                                                                            'y'] + self.vert_spacing,
                                                                        z_1)))
            cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(section_tmp2_path[max_ind]['x'],
                                                                        section_tmp2_path[max_ind][
                                                                            'y'] + self.vert_spacing,
                                                                        z_2)))
            # Reset prev Points
            prev_point_1 = prim.Point(section_tmp1_path[max_ind]['x'],
                                      section_tmp1_path[max_ind]['y'] + self.vert_spacing, z_1)
            prev_point_2 = prim.Point(section_tmp2_path[max_ind]['x'],
                                      section_tmp2_path[max_ind]['y'] + self.vert_spacing, z_2)

            # Move down from the vertically offset point to the Overlapped CrossSection
            cut_list_1.append(
                prim.SectionLink(prev_point_1,
                                 prim.Point(section_tmp1_path[max_ind]['x'], section_tmp1_path[max_ind]['y'], z_1)))
            cut_list_2.append(
                prim.SectionLink(prev_point_2,
                                 prim.Point(section_tmp2_path[max_ind]['x'], section_tmp2_path[max_ind]['y'], z_2)))

            # Cut the entirety of the Overlapped CrossSection
            prev_point_1, prev_point_2 = SpatialPlacement.add_section_vert(cut_list_1, cut_list_2, section_tmp.section1,
                                                                           section_tmp.section2, z_1, z_2)

            # Move from the top of the overlapped CrossSection up to the vertically offset section
            cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(section_tmp1_path[max_ind]['x'],
                                                                        section_tmp1_path[max_ind][
                                                                            'y'] + self.vert_spacing,
                                                                        z_1)))
            cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(section_tmp2_path[max_ind]['x'],
                                                                        section_tmp2_path[max_ind][
                                                                            'y'] + self.vert_spacing,
                                                                        z_2)))
            # Reset the prev points
            prev_point_1 = prim.Point(section_tmp1_path[max_ind]['x'],
                                      section_tmp1_path[max_ind]['y'] + self.vert_spacing, z_1)
            prev_point_2 = prim.Point(section_tmp2_path[max_ind]['x'],
                                      section_tmp2_path[max_ind]['y'] + self.vert_spacing, z_2)

        # Return to the bottom of the primary section
        cut_list_1.append(
            prim.SectionLink(prev_point_1, prim.Point(main_section_1[min_ind]['x'], main_section_1[min_ind]['y'], z_1)))
        cut_list_2.append(
            prim.SectionLink(prev_point_2, prim.Point(main_section_2[min_ind]['x'], main_section_2[min_ind]['y'], z_2)))

        # Return from the bottom of the main section to the top to resume the rest of the cutpath generation
        prev_point_1, prev_point_2 = SpatialPlacement.add_section(cut_list_1, cut_list_2, False, main_section_1,
                                                                  main_section_2, z_1, z_2, dimension='y')

        for item in overlap[entry]['above']:
            section_tmp = self.state_dict[item]['section']
            section_tmp1_path = section_tmp.section1.get_path()
            section_tmp2_path = section_tmp.section2.get_path()
            max_ind = prim.GeometricFunctions.get_index_max_coord(section_tmp1_path, 'y')
            min_ind = prim.GeometricFunctions.get_index_min_coord(section_tmp1_path, 'y')

            # Move from the bottom of the primary CrossSection to the next overlapped CrossSection + Vert spacing
            cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(section_tmp1_path[min_ind]['x'],
                                                                        section_tmp1_path[min_ind][
                                                                            'y'] - self.vert_spacing,
                                                                        z_1)))
            cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(section_tmp2_path[min_ind]['x'],
                                                                        section_tmp2_path[min_ind][
                                                                            'y'] - self.vert_spacing,
                                                                        z_2)))
            # Reset prev Points
            prev_point_1 = prim.Point(section_tmp1_path[min_ind]['x'],
                                      section_tmp1_path[min_ind]['y'] - self.vert_spacing, z_1)
            prev_point_2 = prim.Point(section_tmp2_path[min_ind]['x'],
                                      section_tmp2_path[min_ind]['y'] - self.vert_spacing, z_2)

            # Move down from the vertically offset point to the Overlapped CrossSection
            cut_list_1.append(
                prim.SectionLink(prev_point_1,
                                 prim.Point(section_tmp1_path[min_ind]['x'], section_tmp1_path[min_ind]['y'], z_1)))
            cut_list_2.append(
                prim.SectionLink(prev_point_2,
                                 prim.Point(section_tmp2_path[min_ind]['x'], section_tmp2_path[min_ind]['y'], z_2)))

            # Cut the entirety of the Overlapped CrossSection
            prev_point_1, prev_point_2 = SpatialPlacement.add_section_vert(cut_list_1, cut_list_2, section_tmp.section1,
                                                                           section_tmp.section2, z_1, z_2, bottom=True)

            # Move from the top of the overlapped CrossSection up to the vertically offset section
            cut_list_1.append(prim.SectionLink(prev_point_1, prim.Point(section_tmp1_path[min_ind]['x'],
                                                                        section_tmp1_path[min_ind][
                                                                            'y'] - self.vert_spacing,
                                                                        z_1)))
            cut_list_2.append(prim.SectionLink(prev_point_2, prim.Point(section_tmp2_path[min_ind]['x'],
                                                                        section_tmp2_path[min_ind][
                                                                            'y'] - self.vert_spacing,
                                                                        z_2)))
            # Reset the prev points
            prev_point_1 = prim.Point(section_tmp1_path[min_ind]['x'],
                                      section_tmp1_path[min_ind]['y'] - self.vert_spacing, z_1)
            prev_point_2 = prim.Point(section_tmp2_path[min_ind]['x'],
                                      section_tmp2_path[min_ind]['y'] - self.vert_spacing, z_2)

        max_ind = prim.GeometricFunctions.get_index_max_coord(main_section_1, 'y')
        # Return to the top of the primary section
        cut_list_1.append(
            prim.SectionLink(prev_point_1, prim.Point(main_section_1[max_ind]['x'], main_section_1[max_ind]['y'], z_1)))
        cut_list_2.append(
            prim.SectionLink(prev_point_2, prim.Point(main_section_2[max_ind]['x'], main_section_2[max_ind]['y'], z_2)))

        prev_point_1 = prim.Point(main_section_1[max_ind]['x'], main_section_1[max_ind]['y'], z_1)
        prev_point_2 = prim.Point(main_section_2[max_ind]['x'], main_section_2[max_ind]['y'], z_2)

        return prev_point_1, prev_point_2

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

    @staticmethod
    def get_hole_index_order(holes, ind_list_holes):
        """
        returns three lists defining the sorted order for cutting embedded holes.

        :param list[list[prim.Point]] holes: List of hole data.
        :param list[list[int,int]] ind_list_holes:
        :return:
        """
        hole_order_list = list()
        ind_hole = list()
        ind_primary = list()

        for ind in range(0, len(holes)):
            min_ind = sys.maxsize
            ind_to_use = 0
            for ind2 in range(0, len(ind_list_holes)):
                if min_ind > ind_list_holes[ind2][0] and ind2 not in hole_order_list:
                    min_ind = ind_list_holes[ind2][0]
                    ind_to_use = ind2
            hole_order_list.append(ind_to_use)
            ind_primary.append(min_ind)
            ind_hole.append(ind_list_holes[ind_to_use][1])

        return hole_order_list, ind_primary, ind_hole

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

    def _get_overlap_items(self, cut_piece):
        """
        Checks all cross sections along a single y-axis row against one another and checks if there is any relative
        overlap between them. This information is used to alter the relative cut order to prevent cut intersections
        during section transition.

        :return: Dictionary with each section as an index with all other overlapping sections as the dictionary items.
        """
        overlapped_items = dict()
        for row in self.section_order[cut_piece]:
            for item in row:
                for item2 in row:
                    if item != item2:
                        bb1 = self.state_dict[item]['bb']
                        bb2 = self.state_dict[item2]['bb']

                        overlap = self._bb_overlap(bb1, bb2)
                        if overlap != self.NO_OVERLAP:
                            if item not in overlapped_items:
                                overlapped_items[item] = {'above': list(), 'below': list()}
                            if overlap == self.ABOVE:
                                overlapped_items[item]['above'].append(item2)
                            else:
                                overlapped_items[item]['below'].append(item2)

        return overlapped_items

    def _bb_overlap(self, bb1, bb2):
        """
        Checks for the relative overlap between bb1 and bb2. Returns self.ABOVE if bb2 is above bb1 with x overlap or
        self.BELOW if bb2 is below bb1. Returns self.NO_OVERLAP if there is no overlap.

        :param bb1: Bounding box for item 1 being checked.
        :param bb2: Bounding box for item 2 being checked
        :return: Enum with information about the relative overlap of the two bounding boxes.
        :rtype: int
        """
        ret_val = self.NO_OVERLAP
        p1 = prim.Point(sum([bb1[0][0], bb1[1][0]]) / 2.0, sum([bb1[0][1], bb1[1][1]]) / 2.0, 0)
        p2 = prim.Point(sum([bb2[0][0], bb2[1][0]]) / 2.0, sum([bb2[0][1], bb2[1][1]]) / 2.0, 0)
        if (bb1[0][0] <= p2['x'] <= bb1[1][0]) or (bb2[0][0] <= p1['x'] <= bb2[1][0]):
            if p2['y'] < p1['y']:
                ret_val = self.BELOW
            else:
                ret_val = self.ABOVE

        return ret_val

    def _clean_overlap_dict(self, overlap_dict, cut_piece):
        new_overlap_dict = dict()
        for ind in overlap_dict:
            for ind2 in overlap_dict[ind]['above']:
                if ind2 in overlap_dict and ((len(overlap_dict[ind2]['above']) + len(overlap_dict[ind2]['below'])) <=
                                             (len(overlap_dict[ind]['above']) + len(overlap_dict[ind]['below']))):
                    new_overlap_dict[ind] = overlap_dict[ind]
            for ind2 in overlap_dict[ind]['below']:
                if ind2 in overlap_dict and ((len(overlap_dict[ind2]['above']) + len(overlap_dict[ind2]['below'])) <=
                                             (len(overlap_dict[ind]['above']) + len(overlap_dict[ind]['below']))):
                    new_overlap_dict[ind] = overlap_dict[ind]

        index_to_remove = list()
        for ind in new_overlap_dict:

            start_y = 0
            for row in self.section_order[cut_piece]:
                if ind in row:
                    start_y = self.state_dict[row[0]]['y_pos']
                    break

            if ind in index_to_remove:
                continue
            for ind2 in new_overlap_dict:
                if ind in new_overlap_dict[ind2]['above'] or ind in new_overlap_dict[ind2]['below']:
                    y_ind1 = self.state_dict[ind]['y_pos']
                    y_ind2 = self.state_dict[ind2]['y_pos']
                    if abs(y_ind2 - start_y) < abs(y_ind1 - start_y):
                        index_to_remove.append(ind)
                    else:
                        index_to_remove.append(ind2)

        for ind in list(set(index_to_remove)):
            del new_overlap_dict[ind]

        return new_overlap_dict

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
                plt.savefig(os.path.join(output_dir, 'Cross_Section_cut_order_%s.png' % cut_section))

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
