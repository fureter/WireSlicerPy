import copy
import logging
import os
import timeit

import numpy as np
import trimesh as tm
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import geometry.primative as prim
from geometry.spatial_manipulation import PointManip
from slicer.spatial_placement import SpatialPlacement
from slicer.wire_cutter import WireCutter
from util import util_functions


class CrossSectionPair():
    """ Contains a Pair of CrossSections that are related together spatially. Used to transform CrossSection pair
     uniformly, an turn into CutPaths.

    :param CrossSection section1:
    :param CrossSection section2:
    """
    def __init__(self, section1, section2):
        self.section1 = copy.deepcopy(section1)
        self.section2 = copy.deepcopy(section2)

    def subdivide(self, num_sections):
        """

        :param int num_sections:
        :return:
        """
        ret_val = list()
        if num_sections < 2:
            ret_val = [self]
        else:

            center = prim.Point(0, 0, 0)
            points1, points2 = self.get_path()

            start_point1 = points1[0]

            for point in points1:
                center += point
            center /= len(points1)
            hole1, hole2 = self.section1.get_path_hole(), self.section2.get_path_hole()

            for index in range(num_sections):
                curr_section1 = list()
                curr_section2 = list()
                arc_len = 360/num_sections
                upper_range = arc_len * (index+1)
                lower_range = arc_len * index
                radius = 10000

                point_lower = prim.Point(center['x'] + radius*np.cos(lower_range+180),
                                         center['y'] + radius*np.sin(lower_range+180),
                                         center['z'])

                point_upper = prim.Point(center['x'] + radius*np.cos(upper_range+180),
                                         center['y'] + radius*np.sin(upper_range+180),
                                         center['z'])

                line_lower = prim.Line.line_from_points(center, point_lower)
                line_upper = prim.Line.line_from_points(center, point_upper)
                prev_point = copy.deepcopy(points1[-1])
                for point in points1:
                    theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                    # CrossSectionPair.handle_intersections(curr_section1, prev_point, point, line_lower, line_upper)
                    prev_point = copy.deepcopy(point)

                    if lower_range < theta <= upper_range:
                        curr_section1.append(copy.deepcopy(point))
                # output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'plots', 'Impulse_Reaction')
                # plt.close('all')
                # plt.figure()
                # prim.GeometricFunctions.plot_path(curr_section1, color=None, scatter=True)
                # plt.axis('equal')
                # plt.savefig(os.path.join(output_dir, 'Curr_Section1_%s.png' % index))
                # plt.show()

                if hole1 is not None:
                    tmp_list = list()
                    prev_point = copy.deepcopy(hole1[-1])
                    for point in hole1:
                        theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                        # CrossSectionPair.handle_intersections(tmp_list, prev_point, point, line_lower, line_upper)
                        prev_point = copy.deepcopy(point)

                        if lower_range < theta <= upper_range:
                            tmp_list.append(copy.deepcopy(point))
                    curr_section1 = PointManip.reorder_2d_cw_subdivide(outer_points=copy.deepcopy(curr_section1),
                                                                       inner_points=tmp_list, center=center)
                else:
                    curr_section1.append(copy.deepcopy(center))
                    curr_section1 = PointManip.reorder_2d_cw(copy.deepcopy(curr_section1), method=3)

                prev_point = copy.deepcopy(points2[-1])
                for point in points2:
                    theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                    # CrossSectionPair.handle_intersections(curr_section2, prev_point, point, line_lower, line_upper)
                    prev_point = copy.deepcopy(point)

                    if lower_range < theta <= upper_range:
                        curr_section2.append(copy.deepcopy(point))
                if hole2 is not None:
                    tmp_list = list()
                    prev_point = copy.deepcopy(hole2[-1])
                    for point in hole2:
                        theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                        # CrossSectionPair.handle_intersections(tmp_list, prev_point, point, line_lower, line_upper)
                        prev_point = copy.deepcopy(point)

                        if lower_range < theta <= upper_range:
                            tmp_list.append(copy.deepcopy(point))
                    print('Ordering inner hole of section2 of cross section %s' % index)
                    curr_section2 = PointManip.reorder_2d_cw_subdivide(outer_points=copy.deepcopy(curr_section2),
                                                                       inner_points=tmp_list, center=center)
                else:
                    curr_section2.append(copy.deepcopy(center))
                    curr_section2 = PointManip.reorder_2d_cw(copy.deepcopy(curr_section2), method=3)

                curr_section1 = prim.GeometricFunctions.remove_duplicate_memory_from_path(curr_section1)
                curr_section2 = prim.GeometricFunctions.remove_duplicate_memory_from_path(curr_section2)

                # curr_section1 = PointManip.reorder_2d_cw(curr_section1, method=4)
                # curr_section2 = PointManip.reorder_2d_cw(curr_section2, method=4)

                # if start_point1 in curr_section1:
                #     path1 = prim.GeometricFunctions.close_path(PointManip.reorder_2d_cw(curr_section1, method=2))
                #     path2 = prim.GeometricFunctions.close_path(PointManip.reorder_2d_cw(curr_section2, method=2))
                # else:
                path1 = prim.GeometricFunctions.close_path(
                    prim.GeometricFunctions.normalize_path_points(curr_section1, num_points=256))
                path2 = prim.GeometricFunctions.close_path(
                    prim.GeometricFunctions.normalize_path_points(curr_section2, num_points=256))

                ret_val.append(CrossSectionPair(section1=CrossSection(
                    prim.Path(path1)), section2=CrossSection(prim.Path(path2))))

        return ret_val

    @staticmethod
    def handle_intersections(section, prev_point, point, line_lower, line_upper):
        # Check for intersections with the divider lines, add the intersecting point to the path
        # pass
        line_test = prim.Line.line_from_points(prev_point, point)
        intersects_lower, point_inter_lower = line_test.intersects(line_lower)
        intersects_upper, point_inter_upper = line_test.intersects(line_upper)
        if intersects_lower:
            # pass
            # plt.close('all')
            # plt.figure()
            # line_test.plot()
            # line_lower.plot()
            # line_upper.plot()
            # prim.GeometricFunctions.plot_path(section, scatter=True, color=False)
            # plt.show()
            section.append(copy.deepcopy(point_inter_lower))
        if intersects_upper:
            # pass
            # plt.close('all')
            # plt.figure()
            # line_test.plot()
            # line_lower.plot()
            # line_upper.plot()
            # prim.GeometricFunctions.plot_path(section, scatter=True, color=False)
            # plt.show()
            section.append(copy.deepcopy(point_inter_upper))

    @staticmethod
    def subdivide_list(num_sections, section_list):
        """

        :param num_sections:
        :param list[CrossSectionPair] section_list:
        :return:
        """
        subdivided_list = list()
        for section_pair in section_list:
            sections = section_pair.subdivide(num_sections)
            subdivided_list.extend(sections)
            for section in sections:
                assert len(section.get_path()) > 1, 'Error: Subdivided section has no points'

        return subdivided_list

    def plot_subdivide_debug(self, num_sections, radius):
        center = prim.Point(0, 0, 0)
        points1, _ = self.get_path()
        for point in points1:
            center += point
        center /= len(points1)
        self.section1.plot(color=None, scatter=True)
        self.section2.plot(color=None, scatter=True)
        for index in range(num_sections):
            arc_len = 360/num_sections
            upper_range = arc_len * (index+1) - 180
            plt.plot([center['x'], center['x'] + np.cos(np.deg2rad(upper_range))*radius],
                     [center['y'], center['y'] + np.sin(np.deg2rad(upper_range))*radius])

    def get_path(self):
        return self.section1.get_path(), self.section2.get_path()


class CrossSection():
    """
    Contains a list of sections. Sections are any class that defines a get_path function. Valid CrossSections must
    contain at least one section, and the end of one section must be the start of the next section. Also contains Holes
    which are CrossSections that are internal to the current CrossSection
    """

    def __init__(self, section_list):
        if not isinstance(section_list, list):
            section_list = [section_list]
        self._section_list = section_list
        self.holes = None

    def __getitem__(self, item):
        if item < len(self._section_list):
            return self._section_list[item]
        else:
            raise ValueError('Error Index of %s out of range of length %s' % (item, len(self._section_list)))

    def add_section(self, section):
        self._section_list.append(section)

    def is_valid_cross_section(self):
        ret_val = True
        for indx in range(0, len(self.section_list) - 1):
            if self.section_list[indx + 1][0] != self.section_list[indx][-1]:
                ret_val = False

        # A cross section is valid if it is a single open entry.
        if len(self.section_list) > 1 and self.section_list[-1][-1] != self.section_list[0][0]:
            ret_val = False

        return ret_val

    def get_path(self):
        path = list()
        for section in self.section_list:
            path.extend(section.get_path())
        return path

    def get_path_hole(self):
        ret_val = None
        if self.holes is not None:
            path = list()
            for hole in self.holes:
                path.extend(hole.get_path())
            ret_val = path
        return ret_val

    def plot(self, color=None, show=True, scatter=False):
        logger = logging.getLogger(__name__)
        prim.GeometricFunctions.plot_path(self.get_path(), color=color, scatter=scatter)
        if self.holes is not None:
            logger.debug('hole_path: %s', self.get_path_hole())
            prim.GeometricFunctions.plot_path(self.get_path_hole(), color=color, scatter=scatter)

    def translate(self, vector):
        PointManip.Transform.translate(self.get_path(), vector=vector)
        if self.holes is not None:
            PointManip.Transform.translate(self.get_path_hole(), vector=vector)

    def point_in_section(self, point):
        """
        Uses the even-odd rule algorithm to check whether a point lies within the CrossSection.

        :param point:
        :return: Boolean indicator of whether the point is within the section.
        :rtype: bool
        """
        path = self.get_path()
        bound_box = prim.GeometricFunctions.get_bounding_box_from_path(path)
        if ((point['x'] < bound_box[0][0] or bound_box[1][0] < point['x'])
                or (point['y'] < bound_box[0][1] or bound_box[1][1] < point['y'])):
            ret_val = False
        else:
            max_y_p, _ = prim.GeometricFunctions.get_point_from_max_coord(path, 'y')
            min_y_p = prim.GeometricFunctions.get_point_from_min_coord(path, 'y')

            test_line = prim.Line.line_from_points(prim.Point(point['x'], point['y'], 0),
                                                   prim.Point(point['x'], min_y_p['y']-1, 0))

            times_intersected = 0
            for ind in range(0, len(path)-1):
                point_1 = prim.Point(path[ind]['x'], path[ind]['y'], 0)
                point_2 = prim.Point(path[ind+1]['x'], path[ind+1]['y'], 0)
                tmp_line = prim.Line.line_from_points(point_1, point_2)
                #test_line.plot()
                #tmp_line.plot()
                #plt.show()
                if test_line.intersects(tmp_line)[0]:
                    times_intersected += 1
            ret_val = (times_intersected % 2) == 1

        return ret_val

    def add_simple_hole_from_offset(self, thickness):
        """

        :param float or int thickness: wall_thickenss in mm
        :return:
        """
        logger = logging.getLogger(__name__)
        # path = prim.Path(prim.GeometricFunctions.offset_curve(self.get_path(), thickness, dir=1, divisions=1))
        path = copy.deepcopy(self.get_path())
        path = prim.GeometricFunctions.clean_intersections(
            prim.GeometricFunctions.parallel_curve(path, thickness, 1, False), path, thickness)
        path = prim.GeometricFunctions.close_path(path)
        self.holes = [CrossSection(prim.Path(path))]

    @property
    def section_list(self):
        return self._section_list


class CutPath():
    """
    Contains a list of `CrossSections` and `SectionLinks` that define the work surface cut movement for the gantry. This
    class directly feeds into ToolPath creation.
    """

    def __init__(self, cut_list_1, cut_list_2):
        if len(cut_list_1) != len(cut_list_2):
            raise AttributeError('Error: Cut lists must have the same number of cut segments')
        self._cut_list_1 = cut_list_1
        self._cut_list_2 = cut_list_2

    @property
    def cut_list_1(self):
        return self._cut_list_1

    @property
    def cut_list_2(self):
        return self._cut_list_2

    def is_valid_cut_path(self):
        logger = logging.getLogger(__name__)
        ret_val = True
        l1 = len(self._cut_list_1)
        l2 = len(self._cut_list_2)
        if l1 != l2:
            logger.error('Error: cut_list lengths do not match [L1: %s, L2: %s]' % (l1, l2))
            ret_val = False
        else:
            for indx in range(0, l1 - 1):
                if len(self.cut_list_1[indx + 1].get_path()) < 1:
                    logger.error('Error: Path of section %s has no points' % (indx+1))
                    ret_val = False
                if len(self._cut_list_1[indx].get_path()) < 1:
                    logger.error('Error: Path of section %s has no points' % indx)
                    ret_val = False
                # logger.debug('Index: %s, index+1: %s', indx, indx + 1)
                # logger.debug('cut_list_1[indx]: %s', self._cut_list_1[indx])
                # logger.debug('cut_list_1[indx+1]: %s', self._cut_list_1[indx + 1])
                # logger.debug('cut_list_1[indx].path: %s', self._cut_list_1[indx].get_path())
                # logger.debug('cut_list_1[indx+1].path: %s', self._cut_list_1[indx + 1].get_path())
                # logger.debug('cut_list_1[indx].path[-1]: %s', self._cut_list_1[indx].get_path()[-1])
                # logger.debug('cut_list_1[indx].path[-1]: %s', self._cut_list_1[indx].get_path()[-1])
                # logger.debug('cut_list_1[indx+1].path[0]: %s', self._cut_list_1[indx + 1].get_path()[0])
                if self.cut_list_1[indx + 1].get_path()[0] != self._cut_list_1[indx].get_path()[-1]:
                    logger.error('Error: End of a Section is not the start of another (%s != %s)' %
                                 (self.cut_list_1[indx + 1].get_path()[0], self._cut_list_1[indx].get_path()[-1]))
                    ret_val = False
                if self.cut_list_2[indx + 1].get_path()[0] != self._cut_list_2[indx].get_path()[-1]:
                    logger.error('Error: End of a Section is not the start of another (%s != %s)' %
                                 (self.cut_list_2[indx + 1].get_path()[0], self._cut_list_2[indx].get_path()[-1]))
                    ret_val = False

        return ret_val

    def add_segment_to_cut_lists(self, segment_1, segment_2):
        self._cut_list_1.append(segment_1)
        self._cut_list_2.append(segment_2)

    @staticmethod
    def _add_loopback(cut_path, wire_cutter, root_z, tip_z):
        logger = logging.getLogger(__name__)

        offset = prim.Point(wire_cutter.start_depth, 0, 0)
        cut_path.add_section_link_from_offset(cut_1_offset=offset, cut_2_offset=offset)

        offset = prim.Point(0, wire_cutter.release_height, 0)
        cut_path.add_section_link_from_offset(cut_1_offset=offset, cut_2_offset=offset)

        start1, start2 = cut_path.get_next_start_points()
        next_point1 = prim.Point(0, start1['y'], root_z)
        next_point2 = prim.Point(0, start2['y'], tip_z)
        cut_path.add_section_link_from_abs_coord(point1=next_point1, point2=next_point2, fast_cut=True)

        next_point1 = prim.Point(0, wire_cutter.start_height, root_z)
        next_point2 = prim.Point(0, wire_cutter.start_height, tip_z)
        cut_path.add_section_link_from_abs_coord(point1=next_point1, point2=next_point2, fast_cut=True)

        return next_point1, next_point2

    def add_section_link_from_offset(self, cut_1_offset, cut_2_offset, fast_cut=False):
        """

        :param prim.Point cut_1_offset:
        :param prim.Point cut_2_offset:
        :return:
        """
        start1, start2 = self.get_next_start_points()
        self.add_segment_to_cut_lists(
            prim.SectionLink(start_point=start1, end_point=start1 + cut_1_offset, fast_cut=fast_cut),
            prim.SectionLink(start_point=start2, end_point=start2 + cut_2_offset, fast_cut=fast_cut))

    def add_section_link_from_abs_coord(self, point1, point2, fast_cut=False):
        """

        :param prim.Point point1:
        :param prim.Point point2:
        :return:
        """
        start1, start2 = self.get_next_start_points()
        self.add_segment_to_cut_lists(
            prim.SectionLink(start_point=start1, end_point=point1, fast_cut=fast_cut),
            prim.SectionLink(start_point=start2, end_point=point2, fast_cut=fast_cut))

    @staticmethod
    def create_cut_path_from_wing(wing, wire_cutter, debug_kerf=False):
        """

        :param WingSegment wing:
        :param WireCutter wire_cutter:
        :return:
        """
        logger = logging.getLogger(__name__)

        if not wing.prepped:
            raise ValueError('Error: Wing has not been prepped for slicing')

        cut_path = CutPath([], [])
        root_foil = copy.deepcopy(wing.root_airfoil)
        tip_foil = copy.deepcopy(wing.tip_airfoil)

        if wire_cutter.kerf is not None:
            if wing.root_chord > wing.tip_chord:
                kerf_root = wire_cutter.kerf
                kerf_tip = wire_cutter.kerf * wing.root_chord/wing.tip_chord * 1.6
            else:
                kerf_root = wire_cutter.kerf * wing.root_chord/wing.tip_chord * 1.6
                kerf_tip = wire_cutter.kerf
            if debug_kerf:
                prim.GeometricFunctions.plot_path(root_foil, 1)
                prim.GeometricFunctions.plot_path(tip_foil, 2)

            root_foil = prim.GeometricFunctions.offset_curve(root_foil, kerf_root, dir=0, divisions=2)
            tip_foil = prim.GeometricFunctions.offset_curve(tip_foil, kerf_tip, dir=0, divisions=2)

            leading_edge_root = prim.GeometricFunctions.get_point_from_min_coord(root_foil, 'x')
            leading_edge_tip = prim.GeometricFunctions.get_point_from_min_coord(tip_foil, 'x')

            offset = leading_edge_root['x'] if leading_edge_root['x'] < leading_edge_tip['x'] else leading_edge_tip['x']
            if offset < 0:
                PointManip.Transform.translate(root_foil, [-offset, 0, 0])
                PointManip.Transform.translate(tip_foil, [-offset, 0, 0])

            if debug_kerf:
                prim.GeometricFunctions.plot_path(root_foil, 3)
                prim.GeometricFunctions.plot_path(tip_foil, 4)

                plt.legend(['Root Foil', 'Tip Foil', 'Root Foil with %smm kerf' % kerf_root,
                            'Tip Foil with %smm kerf' % kerf_tip])
                plt.axis('equal')
                plt.show()

        root_z = root_foil[0]['z']
        tip_z = tip_foil[0]['z']
        logger.debug('Root: %s | Tip: %s' % (root_foil[0], tip_foil[0]))

        start_point1 = prim.Point(0, 0, root_z)
        start_point2 = prim.Point(0, 0, tip_z)
        next_point1 = start_point1 + prim.Point(0, wire_cutter.start_height, 0)
        next_point2 = start_point2 + prim.Point(0, wire_cutter.start_height, 0)
        logger.debug('sp1: %s | sp2: %s | np1: %s | np2: %s' % (start_point1, start_point2, next_point1, next_point2))

        seg_link1 = prim.SectionLink(start_point1, next_point1, fast_cut=True)
        seg_link2 = prim.SectionLink(start_point2, next_point2, fast_cut=True)

        cut_path.add_segment_to_cut_lists(segment_1=seg_link1, segment_2=seg_link2)

        start_point1 = next_point1
        start_point2 = next_point2
        next_point1 = start_point1 + prim.Point(wire_cutter.start_depth, 0, 0)
        # The start depth of the second axis needs to be offset by the difference between the two foils positioning,
        # this is to account for sweep in the wing
        next_point2 = start_point2 + prim.Point(wire_cutter.start_depth - (root_foil[0]['x'] - tip_foil[0]['x']), 0, 0)
        logger.debug('sp1: %s | sp2: %s | np1: %s | np2: %s' % (start_point1, start_point2, next_point1, next_point2))

        test_line = prim.Line.line_from_points(next_point1, next_point2)
        p1 = test_line.get_extrapolated_point(0, constraint_dim='z')
        p2 = test_line.get_extrapolated_point(wire_cutter.wire_length, constraint_dim='z')

        if p1['x'] < wire_cutter.start_depth or p2['x'] < wire_cutter.start_depth:
            offset = max(wire_cutter.start_depth - p1['x'], wire_cutter.start_depth-p2['x'])
            next_point1 += prim.Point(offset, 0, 0)
            next_point2 += prim.Point(offset, 0, 0)

        # Translate both airfoils by the same offset to keep them inline
        logger.debug('Next Point X: %s', next_point1['x'])
        PointManip.Transform.translate(root_foil, [next_point1['x'], next_point1['y'], 0])
        PointManip.Transform.translate(tip_foil, [next_point1['x'], next_point1['y'], 0])
        logger.debug('Root Airfoil Leading Edge: %s', root_foil[0])

        next_point1 += root_foil[0] - next_point1
        logger.debug('Next Point X after offset: %s', next_point1['x'])
        next_point2 += tip_foil[0] - next_point2
        logger.debug('Root Airfoil Leading Edge: %s', root_foil[0])

        seg_link1 = prim.SectionLink(start_point1, next_point1, fast_cut=False)
        seg_link2 = prim.SectionLink(start_point2, next_point2, fast_cut=False)

        cut_path.add_segment_to_cut_lists(segment_1=seg_link1, segment_2=seg_link2)

        root_ind_split = prim.GeometricFunctions.get_index_max_coord(root_foil, 'x')
        tip_ind_split = prim.GeometricFunctions.get_index_max_coord(tip_foil, 'x')

        top_root = root_foil[0:root_ind_split + 1]
        bottom_root = [root_foil[0]]
        bottom_root.extend(sorted(root_foil[root_ind_split:-1], key=lambda point: point['x']))

        top_tip = tip_foil[0:tip_ind_split + 1]
        bottom_tip = [tip_foil[0]]
        bottom_tip.extend(sorted(tip_foil[tip_ind_split:-1], key=lambda point: point['x']))

        cut_path.add_segment_to_cut_lists(CrossSection(prim.Path(top_root)), CrossSection(prim.Path(top_tip)))

        start_point1 = top_root[-1]
        start_point2 = top_tip[-1]
        logger.debug('TE of Top Root: %s | TE of Top Tip: %s' % (start_point1, start_point2))
        next_point1, next_point2 = CutPath._add_loopback(cut_path, wire_cutter, root_z,
                                                         tip_z)

        start_point1 = next_point1
        start_point2 = next_point2
        next_point1 = start_point1 + prim.Point(wire_cutter.start_depth, 0, 0)
        next_point2 = start_point2 + prim.Point(wire_cutter.start_depth - (root_foil[0]['x'] - tip_foil[0]['x']), 0, 0)
        logger.debug('sp1: %s | sp2: %s | np1: %s | np2: %s' % (start_point1, start_point2, next_point1, next_point2))

        test_line = prim.Line.line_from_points(next_point1, next_point2)
        p1 = test_line.get_extrapolated_point(0, constraint_dim='z')
        p2 = test_line.get_extrapolated_point(wire_cutter.wire_length, constraint_dim='z')

        if p1['x'] < wire_cutter.start_depth or p2['x'] < wire_cutter.start_depth:
            offset = max(wire_cutter.start_depth - p1['x'], wire_cutter.start_depth-p2['x'])
            next_point1 += prim.Point(offset, 0, 0)
            next_point2 += prim.Point(offset, 0, 0)

        next_point1 += root_foil[0] - next_point1
        logger.debug('Next Point X after offset: %s', next_point1['x'])
        next_point2 += tip_foil[0] - next_point2
        logger.debug('Root Airfoil Leading Edge: %s', root_foil[0])

        seg_link1 = prim.SectionLink(start_point1, next_point1, fast_cut=False)
        seg_link2 = prim.SectionLink(start_point2, next_point2, fast_cut=False)

        cut_path.add_segment_to_cut_lists(segment_1=seg_link1, segment_2=seg_link2)

        cut_path.add_segment_to_cut_lists(CrossSection(prim.Path(bottom_root)), CrossSection(prim.Path(bottom_tip)))

        CutPath._add_loopback(cut_path, wire_cutter, root_z, tip_z)

        return cut_path

    @staticmethod
    def create_cut_path_from_cross_section_pair_list(cross_section_pairs, work_piece, wire_cutter, output_dir):
        """

        :param list[CrossSectionPair] cross_section_pairs:
        :return: Returns a CutPath created from the cross_section_pairs
        :rtype: CutPath
        """
        logger = logging.getLogger(__name__)
        start = timeit.default_timer()
        sp = SpatialPlacement(work_piece, wire_cutter)
        sp.bin_packing_algorithm(cross_section_pairs, output_dir=output_dir, distance_between_sections=5)
        logger.debug('Finished aligning cross sections on workpiece, took %ss', timeit.default_timer() - start)

        # todo: normalize cross section pair lists here?

        cut_path_1, cut_path_2 = sp.create_section_links_for_cross_section_pairs()
        sp.plot_section_order(output_dir)
        sp.plot_section_splitting_debug(output_dir)
        cut_path = CutPath(cut_path_1, cut_path_2)
        cut_path.plot_cut_path(out_dir=output_dir)
        return cut_path

    def plot_section_link_connections(self):
        plt.figure()
        offset = 1.5
        for ind in range(0, len(self._cut_list_1)):
            if isinstance(self.cut_list_1[ind], prim.SectionLink):
                path1 = self.cut_list_1[ind].get_path()
                path2 = self.cut_list_2[ind].get_path()

                x1 = [path1[0]['x'], path1[1]['x']]
                y1 = [path1[0]['y'], path1[1]['y']]
                plt.plot(x1, y1, 'r')

                x2 = [path2[0]['x'] + offset, path2[1]['x'] + offset]
                y2 = [path2[0]['y'] + offset, path2[1]['y'] + offset]
                plt.plot(x2, y2, 'k')

                xc = [path1[0]['x'], path2[0]['x'] + offset]
                yc = [path1[0]['y'], path2[0]['y'] + offset]
                plt.plot(xc, yc, 'g')

                xc = [path1[1]['x'], path2[1]['x'] + offset]
                yc = [path1[1]['y'], path2[1]['y'] + offset]
                plt.plot(xc, yc, 'g')
        plt.show()

    def plot_cut_path(self, out_dir=None, show=True):
        plt.close('all')
        plt.figure(figsize=(16, 9), dpi=320)
        for item in self._cut_list_1:
            path = item.get_path()
            len_path = len(path)
            x = np.zeros(len_path)
            y = np.zeros(len_path)
            for ind in range(0, len_path):
                x[ind] = path[ind]['x']
                y[ind] = path[ind]['y']
            plt.plot(x, y, 'r')

        for item in self._cut_list_2:
            path = item.get_path()
            len_path = len(path)
            x = np.zeros(len_path)
            y = np.zeros(len_path)
            for ind in range(0, len_path):
                x[ind] = path[ind]['x']
                y[ind] = path[ind]['y']
            plt.plot(x, y, 'k')

        plt.axis('equal')
        if show:
            plt.show()
        if out_dir:
            plt.savefig(os.path.join(out_dir, 'cut_path.png'))

    def animate(self, file_path=None):
        path1 = list()
        path2 = list()
        for ind in range(0, len(self._cut_list_1)):
            path1.extend(self._cut_list_1[ind].get_path())
            path2.extend(self._cut_list_2[ind].get_path())

        fig, ax = plt.subplots()
        xdata1, ydata1 = [], []
        ln1, = plt.plot([], [], 'r')
        xdata2, ydata2 = [], []
        ln2, = plt.plot([], [], 'b')

        def init():
            ax.set_xlim(0, 400)
            ax.set_ylim(-20, 400)
            return ln1, ln2,

        def update(frame):
            xdata1.append(path1[frame]['x'])
            ydata1.append(path1[frame]['y'])
            ln1.set_data(xdata1, ydata1)

            xdata2.append(path2[frame]['x'])
            ydata2.append(path2[frame]['y'])
            ln2.set_data(xdata2, ydata2)
            return ln1, ln2,

        ani = FuncAnimation(fig, update, frames=list(range(0, len(path1))),
                            init_func=init, blit=True)
        plt.show()

    def get_next_start_points(self):
        return self._cut_list_1[-1].get_path()[-1], self._cut_list_2[-1].get_path()[-1]


class STL():
    """

    :param str _file_path:
    :param logging.Logger logger:
    :param list[CrossSection] or None cross_sections:
    :param list[tm.Path2D] or None trimesh_cross_section:
    """

    def __init__(self, file_path, units='cm'):

        self.logger = logging.getLogger(__name__)
        self.cross_sections = None
        self.trimesh_cross_sections = None
        self._file_path = file_path
        self._setup(units=units)

    def create_cross_section_pairs(self, wall_thickness, origin_plane, spacing, length, open_nose=False,
                                   open_tail=False):
        """

        :param float wall_thickness:
        :param prim.Plane origin_plane:
        :param float spacing:
        :param int length:
        :param bool open_nose:
        :param bool open_tail:
        :return:
        """
        logger = logging.getLogger(__name__)
        cross_section_pair_list = list()
        self.slice_into_cross_sections(origin_plane, spacing, length)
        cross_section_list = copy.deepcopy(self.cross_sections)

        if open_nose and len(cross_section_list) > 1:
            cross_section_list[0].add_simple_hole_from_offset(wall_thickness)
        if open_tail and len(cross_section_list) > 2:
            cross_section_list[-1].add_simple_hole_from_offset(wall_thickness)

        for ind in range(1, len(cross_section_list)-1):
            if not open_nose and ind == 1:
                cross_section_pair_list.append(CrossSectionPair(cross_section_list[ind-1],
                                                                copy.deepcopy(cross_section_list[ind])))
                cross_section_list[ind].add_simple_hole_from_offset(wall_thickness)
            elif not open_tail and ind == len(cross_section_list)-1:
                cross_section_pair_list.append(CrossSectionPair(cross_section_list[ind-1], cross_section_list[ind]))
            else:
                cross_section_list[ind].add_simple_hole_from_offset(wall_thickness)
                cross_section_pair_list.append(CrossSectionPair(cross_section_list[ind-1], cross_section_list[ind]))
        cross_section_pair_list.append(CrossSectionPair(cross_section_list[-2], cross_section_list[-1]))

        return cross_section_pair_list

    def slice_into_cross_sections(self, origin_plane, spacing, length):
        """

        :param Plane origin_plane:
        :param int length:
        :param float spacing:
        :return:
        :rtype: list[CrossSection]
        """
        self.trimesh_cross_sections = list()
        heights = list()
        for i in range(length):
            heights.append(spacing*i)
        origin = np.array(origin_plane.origin)
        normal = np.array(origin_plane.normal)
        sections = self.mesh.section_multiplane(plane_origin=origin, plane_normal=normal, heights=heights)
        if sections is not None:
            self.trimesh_cross_sections.extend(sections)
            self.cross_sections = list()
            for ind1, section in enumerate(sections):
                points = list()
                for ind in range(0, len(section.discrete[0])):
                    points.append(prim.Point(section.discrete[0][ind][0], section.discrete[0][ind][1], 0))
                plt.close('all')
                plt.figure(figsize=(16, 9), dpi=320)
                prim.GeometricFunctions.plot_path(points, color='C1', scatter=True)
                path = PointManip.reorder_2d_cw(points, method=3)
                path = prim.GeometricFunctions.close_path(path)
                path = prim.GeometricFunctions.normalize_path_points(path, 512*2)
                prim.GeometricFunctions.plot_path(path, color='C2', scatter=True)
                plt.legend(['C1: Before mod', 'C2: After mod'])
                output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'plots', 'Impulse_Reaction')
                plt.axis('equal')
                plt.savefig(os.path.join(output_dir, 'Cross_Section_Debug_%s.png' % ind1))
                plt.show()
                # plt.show()
                path = prim.GeometricFunctions.remove_duplicate_memory_from_path(path)

                self.cross_sections.append(CrossSection(section_list=[prim.Path(path)]))
                self.logger.debug('section: %s', self.cross_sections[-1])
            self.center_cross_sections()
            self.close_sections()
        else:
            raise AttributeError('Error: Plane with origin(%s) does not intersect the STL' % origin_plane.origin)

    def plot_stl(self):
        self.mesh.show()

    def plot_cross_sections(self, show=False):
        logger = logging.getLogger(__name__)
        if self.cross_sections is None:
            raise AttributeError('Error: Cross sections have not been generated for this STL')
        num_sections = len(self.cross_sections)
        logger.info('Number of sections being plotted: %s', num_sections)
        (r, c) = util_functions.get_r_and_c_from_num(num_sections)
        logger.info('Creating section subplots with r: %s and c: %s', r, c)
        i = 1
        for section in self.cross_sections:
            ax = plt.subplot(int(r), int(c), i)
            section.plot(show=False, scatter=False)
            ax.set_title('Cross Section: %s\n' % i)
            ax.set_aspect('equal', 'datalim')
            i += 1
        fig = plt.gcf()
        fig.set_size_inches(16, 9)
        plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.4, hspace=0.5)
        if show:
            plt.show()

    def plot_trimesh_cross_sections(self, show=False):
        logger = logging.getLogger(__name__)
        if self.cross_sections is None:
            raise AttributeError('Error: Cross sections have not been generated for this STL')
        num_sections = len(self.cross_sections)
        logger.info('Number of sections being plotted: %s', num_sections)
        (r, c) = util_functions.get_r_and_c_from_num(num_sections)
        logger.info('Creating section subplots with r: %s and c: %s', r, c)
        i = 1
        for section in self.trimesh_cross_sections:
            ax = plt.subplot(int(r), int(c), i)
            axis = section.plot_discrete(show=False)
            ax.set_title('Cross Section: %s' % i)
            i += 1
        fig = plt.gcf()
        fig.set_size_inches(16, 9)
        plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.95, wspace=0.4, hspace=0.5)
        if show:
            plt.show()

    def center_cross_sections(self):
        center = prim.Point(0, 0, 0)
        for point in self.cross_sections[0].get_path():
            center += point
        center /= len(self.cross_sections[0].get_path())
        for section in self.cross_sections:
            section.translate(vector=[-center['x'], -center['y'], -center['z']])

    def close_sections(self):
        for ind in range(0, len(self.cross_sections)):
            self.cross_sections[ind] = CrossSection(prim.Path(prim.GeometricFunctions.close_path(
                self.cross_sections[ind].get_path())))

    def _setup(self, units):
        self.mesh = tm.load(self._file_path, force='mesh')
        self.mesh.metadata['units'] = units


class WingSegment():
    """

    """

    def __init__(self, name, logger):
        """

        :param name:
        :param logger:
        """
        if logger is None:
            logger = logging.getLogger()
        self.logger = logger

        self.name = name

        self.root_airfoil = None
        self.tip_airfoil = None

        self.span = None
        self.root_chord = None
        self.tip_chord = None

        self.sweep = None
        self.washout = None

        self._rotated = False

        self.symmetric = False

        self.prepped = False

    def prep_for_slicing(self, plot=False):
        """

        :return:
        """
        self._check_minimum_data_present()

        if self.sweep is None:
            self.logger.warning('Sweep is not specified for Wing Segment: [%s],'
                                ' assuming 0° Leading Edge Sweep' % self.name)
            self.sweep = 0

        if self.washout is None:
            self.logger.warning('Washout is not specified for Wing Segment: [%s],'
                                ' assuming no twist in wing segment' % self.name)
            self.washout = 0

        if self.symmetric:
            self.logger.info('Wing Segment [%s] is symmetric, a left and right will be generated' % self.name)

        # Rotation is in reference to the origin, so airfoils need to have the leading edge centered at the origin
        self.align_leading_edges_to_origin()

        # Un-rotate the airfoils if they have already been rotated previously. This needs to be done so that scaling
        # for the chord works properly
        if self._rotated:
            PointManip.Transform.rotate(self.tip_airfoil, [0, 0, -np.deg2rad(self.washout)])
            self._rotated = False

        # Check if the root airfoil chord matches the desired root chord, if not, scale it to match
        _, actual_root_chord = prim.GeometricFunctions.get_point_from_max_coord(self.root_airfoil, 'x')
        if actual_root_chord != self.root_chord:
            scale = self.root_chord / actual_root_chord
            PointManip.Transform.scale(self.root_airfoil, [scale, scale, 1])

        # Check if the tip airfoil chord matches the desired tip chord, if not, scale it to match
        _, actual_tip_chord = prim.GeometricFunctions.get_point_from_max_coord(self.tip_airfoil, 'x')
        if actual_tip_chord != self.tip_chord:
            scale = self.tip_chord / actual_tip_chord
            PointManip.Transform.scale(self.tip_airfoil, [scale, scale, 1])

        # Rotate the tip airfoil by the washout amount, needs to be done before the airfoil is translated any further
        PointManip.Transform.rotate(self.tip_airfoil, [0, 0, np.deg2rad(self.washout)])
        self._rotated = True

        # Translate the tip airfoil back by the sweep amount if the sweep is a positive angle,
        # Translate the root if the sweep is a negative angle
        sweep_offset = self.span * np.sin(np.deg2rad(self.sweep))
        self.logger.info('Offsetting Airfoil by %smm to account for sweep' % sweep_offset)
        if sweep_offset < 0:
            PointManip.Transform.translate(self.root_airfoil, [sweep_offset, 0, 0])
        else:
            PointManip.Transform.translate(self.tip_airfoil, [sweep_offset, 0, 0])

        # Move the tip airfoil to the end of the span
        PointManip.Transform.translate(self.tip_airfoil, [0, 0, self.span])

        if plot:
            x = list()
            y = list()
            for point in self.root_airfoil:
                x.append(point['x'])
                y.append(point['y'])
            plt.plot(x, y)

            x = list()
            y = list()
            for point in self.tip_airfoil:
                x.append(point['x'])
                y.append(point['y'])
            plt.plot(x, y)
            plt.axis('equal')

        self.prepped = True

    def align_leading_edge_with_wire(self):
        """
        rotates the wing so that the leading edge is parallel to the z-axis. Wing has to be prepped for slicing, if the
        wing is not prepped before hand, it will be prepped.
        :return:
        """
        logger = logging.getLogger(__name__)
        if not self.prepped:
            self.prep_for_slicing()

        rot = [0, np.deg2rad(-self.sweep), 0]
        origin = [0, 0, self.root_airfoil[0]['z'] if self.root_airfoil[0]['x'] < self.tip_airfoil[0]['x'] else
                        self.tip_airfoil[0]['z']]
        logger.debug('Origin for rotation: %s', origin)
        PointManip.Transform.rotate(self.root_airfoil, rot, origin)
        PointManip.Transform.rotate(self.tip_airfoil, rot, origin)
        leading_edge_root = prim.GeometricFunctions.get_point_from_min_coord(self.root_airfoil, 'x')
        leading_edge_tip = prim.GeometricFunctions.get_point_from_min_coord(self.tip_airfoil, 'x')
        PointManip.Transform.translate(self.root_airfoil, [-leading_edge_root['x'], 0, 0])
        PointManip.Transform.translate(self.tip_airfoil, [-leading_edge_tip['x'], 0, 0])
        logger.debug('Root Airfoil Leading Edge: %s', self.root_airfoil[0])
        logger.debug('Tip Airfoil Leading Edge: %s', self.tip_airfoil[0])

    def _check_minimum_data_present(self):
        """

        :return:
        """
        if self.span is None:
            raise AttributeError('Error: Span is None for Wing Segment [%s]' % self.name)
        if self.root_airfoil is None:
            raise AttributeError('Error: Root Airfoil is None for Wing Segment [%s]' % self.name)
        if self.tip_airfoil is None:
            raise AttributeError('Error: Tip Airfoil is None for Wing Segment [%s]' % self.name)
        if self.root_chord is None:
            raise AttributeError('Error: Root Chord is None for Wing Segment [%s]' % self.name)
        if self.tip_chord is None:
            raise AttributeError('Error: Tip Chord is None for Wing Segment [%s]' % self.name)

    def align_leading_edges_to_origin(self):
        """

        :return:
        """
        leading_edge_root = prim.GeometricFunctions.get_point_from_min_coord(self.root_airfoil, 'x')
        PointManip.Transform.translate(self.root_airfoil, [-leading_edge_root['x'], -leading_edge_root['y'],
                                                           -leading_edge_root['z']])

        leading_edge_tip = prim.GeometricFunctions.get_point_from_min_coord(self.tip_airfoil, 'x')
        PointManip.Transform.translate(self.tip_airfoil, [-leading_edge_tip['x'], -leading_edge_tip['y'],
                                                          -leading_edge_tip['z']])

    def center_to_wire_cutter(self, wire_cutter):
        """

        :param WireCutter wire_cutter:
        :return:
        """
        if not self.prepped:
            self.logger.warning('Wing Segment has not been prepped for slicing, '
                                'run prep_for_slicing() before centering')
        else:
            point_root = prim.GeometricFunctions.get_point_from_min_coord(self.root_airfoil, 'x')
            point_tip = prim.GeometricFunctions.get_point_from_min_coord(self.tip_airfoil, 'x')

            center = wire_cutter.wire_length / 2
            wing_half = self.span / 2

            if wing_half > center:
                raise AttributeError('Error: Wing does not fit in the wire cutter')

            des_root_pos = center - wing_half
            des_tip_pos = center + wing_half

            print('point tip: %s' % point_tip)
            print('point root: %s' % point_root)
            PointManip.Transform.translate(self.tip_airfoil, [0, 0, des_tip_pos - point_tip['z']])
            PointManip.Transform.translate(self.root_airfoil, [0, 0, des_root_pos - point_root['z']])

    def flip_tip_and_root(self):
        """

        :return:
        """
        root_z = self.root_airfoil[0]['z']
        tip_z = self.tip_airfoil[0]['z']

        PointManip.Transform.translate(self.root_airfoil, [0, 0, tip_z - root_z])
        PointManip.Transform.translate(self.tip_airfoil, [0, 0, root_z - tip_z])

    def set_tip_airfoil(self, airfoil):
        """

        :param list[Point] airfoil:
        :return:
        """
        self.tip_airfoil = copy.deepcopy(airfoil)

    def set_root_airfoil(self, airfoil):
        """

        :param list[Point] airfoil:
        :return:
        """
        self.root_airfoil = copy.deepcopy(airfoil)

    def set_span(self, span):
        """

        :param float span:
        :return:
        """

        self.span = span

    def set_tip_chord(self, chord):
        """

        :param float chord:
        :return:
        """
        self.tip_chord = chord

    def set_root_chord(self, chord):
        """

        :param float chord:
        :return:
        """
        self.root_chord = chord

    def set_sweep(self, sweep):
        """

        :param float sweep:
        :return:
        """
        self.sweep = sweep

    def set_washout(self, washout):
        """

        :param float washout:
        :return:
        """
        self.washout = washout

    def thickness_ratio(self):
        """


        :return:
        """
        root_thickness = prim.GeometricFunctions.get_max_thickness(self.root_airfoil, 'y', 'x')
        tip_thickness = prim.GeometricFunctions.get_max_thickness(self.tip_airfoil, 'y', 'x')

        return root_thickness / self.root_chord, tip_thickness / self.tip_chord

    def planform_coordinates(self):
        len_x = 8 if self.symmetric else 4
        len_y = len_x

        x = np.zeros(len_x)
        y = np.zeros(len_y)

        x[0] = 0
        x[1] = self.span
        x[2] = self.span
        x[3] = 0
        y[0] = self.root_chord
        y[1] = self.root_chord - self.span * np.sin(np.deg2rad(self.sweep))
        y[2] = self.root_chord - self.tip_chord - self.span * np.sin(np.deg2rad(self.sweep))
        y[3] = 0

        if self.symmetric:
            x[4] = 0
            x[5] = -self.span
            x[6] = -self.span
            x[7] = 0
            y[7] = self.root_chord
            y[6] = self.root_chord - self.span * np.sin(np.deg2rad(self.sweep))
            y[5] = self.root_chord - self.tip_chord - self.span * np.sin(np.deg2rad(self.sweep))
            y[4] = 0

        return (np.array(x), np.array(y))

    def plot_planform(self):
        (x, y) = self.planform_coordinates()

        plt.plot(x, y)
        plt.xlabel('y (m)')
        plt.ylabel('x (m)')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis('equal')

    def plot_cut_planform(self):
        leading_edge_root = prim.GeometricFunctions.get_point_from_min_coord(self.root_airfoil, 'x')
        trailing_edge_root, _ = prim.GeometricFunctions.get_point_from_max_coord(self.root_airfoil, 'x')

        leading_edge_tip = prim.GeometricFunctions.get_point_from_min_coord(self.tip_airfoil, 'x')
        trailing_edge_tip, _ = prim.GeometricFunctions.get_point_from_max_coord(self.tip_airfoil, 'x')

        x = [leading_edge_root['z'], leading_edge_tip['z'], trailing_edge_tip['z'], trailing_edge_root['z'],
             leading_edge_root['z']]
        y = [leading_edge_root['x'], leading_edge_tip['x'], trailing_edge_tip['x'], trailing_edge_root['x'],
             leading_edge_root['x']]

        plt.xlabel('Machine Z Axis (Length Along Wire mm)')
        plt.ylabel('Machine X Axis (Length Down Gantry [X, U] mm)')
        plt.plot(x, y)
