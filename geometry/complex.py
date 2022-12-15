import copy
import logging
import os
import sys
import timeit

import numpy as np
import trimesh as tm
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import geometry.primative as prim
import serializer
from geometry.spatial_manipulation import PointManip
from slicer.spatial_placement import SpatialPlacement
from slicer.wire_cutter import WireCutter
from util import util_functions


class CrossSectionPair(object):
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

            for point in points1:
                center += point
            center /= len(points1)
            hole1, hole2 = self.section1.get_path_hole(), self.section2.get_path_hole()

            for index in range(num_sections):
                curr_section1 = list()
                curr_section2 = list()
                arc_len = 360 / num_sections
                upper_range = arc_len * (index + 1)
                lower_range = arc_len * index
                radius = 10000

                for point in points1:
                    theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                    if lower_range < theta <= upper_range:
                        curr_section1.append(copy.deepcopy(point))

                if hole1 is not None:
                    tmp_list = list()
                    for hole in hole1:
                        for point in hole:
                            theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                            if lower_range < theta <= upper_range:
                                tmp_list.append(copy.deepcopy(point))
                    curr_section1 = PointManip.reorder_2d_cw_subdivide(outer_points=copy.deepcopy(curr_section1),
                                                                       inner_points=tmp_list, center=center)
                else:
                    curr_section1.append(copy.deepcopy(center))
                    curr_section1 = PointManip.reorder_2d_cw(copy.deepcopy(curr_section1), method=3)

                for point in points2:
                    theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                    if lower_range < theta <= upper_range:
                        curr_section2.append(copy.deepcopy(point))
                if hole2 is not None:
                    tmp_list = list()
                    for hole in hole2:
                        for point in hole:
                            theta = (np.rad2deg(np.arctan2(point['y'] - center['y'], point['x'] - center['x'])) + 180) % 360

                            if lower_range < theta <= upper_range:
                                tmp_list.append(copy.deepcopy(point))
                    curr_section2 = PointManip.reorder_2d_cw_subdivide(outer_points=copy.deepcopy(curr_section2),
                                                                       inner_points=tmp_list, center=center)
                else:
                    curr_section2.append(copy.deepcopy(center))
                    curr_section2 = PointManip.reorder_2d_cw(copy.deepcopy(curr_section2), method=3)

                curr_section1 = prim.GeometricFunctions.remove_duplicate_memory_from_path(curr_section1)
                curr_section2 = prim.GeometricFunctions.remove_duplicate_memory_from_path(curr_section2)

                path1 = prim.GeometricFunctions.normalize_path_points(prim.GeometricFunctions.close_path(curr_section1),
                                                                      num_points=200)
                path2 = prim.GeometricFunctions.normalize_path_points(prim.GeometricFunctions.close_path(curr_section2),
                                                                      num_points=200)

                ret_val.append(CrossSectionPair(section1=CrossSection(
                    prim.Path(path1)), section2=CrossSection(prim.Path(path2))))

        return ret_val

    @staticmethod
    def handle_intersections(section, prev_point, point, line_lower, line_upper):
        # Check for intersections with the divider lines, add the intersecting point to the path
        line_test = prim.Line.line_from_points(prev_point, point)
        intersects_lower, point_inter_lower = line_test.intersects(line_lower)
        intersects_upper, point_inter_upper = line_test.intersects(line_upper)
        if intersects_lower:
            section.append(copy.deepcopy(point_inter_lower))
        if intersects_upper:
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
            if num_sections > 1:
                sections = section_pair.subdivide(num_sections)
            else:
                sections = [section_pair]
            subdivided_list.extend(sections)
            for section in sections:
                assert len(section.get_path()) > 1, 'Error: Subdivided section has no points'

        return subdivided_list

    def plot_subdivide_debug(self, num_sections, radius, color=None):
        center = prim.Point(0, 0, 0)
        points1, _ = self.get_path()
        for point in points1:
            center += point
        center /= len(points1)
        self.section1.plot(color=color, scatter=True)
        self.section2.plot(color=color, scatter=True)
        for index in range(num_sections):
            arc_len = 360 / num_sections
            upper_range = arc_len * (index + 1) - 180
            plt.plot([center['x'], center['x'] + np.cos(np.deg2rad(upper_range)) * radius],
                     [center['y'], center['y'] + np.sin(np.deg2rad(upper_range)) * radius])

    def plot_gui(self, plot1, font_color, scatter_color1, scatter_color2):
        def plot(plot1):
            size = len(self._data)
            x = np.zeros(size)
            y = np.zeros(size)

            for i in range(0, size):
                x[i] = self._data[i]['x']
                y[i] = self._data[i]['y']
            plot1.plot(x, y, font_color)
            plot1.plot(x, y, 'v', markersize=3, color=scatter_color1)
            plot1.axis('equal')

        return plot

    def apply_kerf(self, kerf, max_kerf):
        """
        Applies the kerf value to the two Cross Sections to prepare for cutting with a wire cutter.

        :param float kerf: Generalized Kerf value. Scales based on the length of the cut path (longer path = lower kerf)
         kerf*path_len [mm^2].
        :param float max_kerf: Maximum value allowed for kerf, bandaid for very small sections that explode the kerf.
        """
        if kerf:
            self.section1.apply_kerf(kerf, max_kerf)
            self.section2.apply_kerf(kerf, max_kerf)

    def align_start(self):
        section1_path = self.section1.get_path()
        section2_path = self.section2.get_path()

        start_index1 = prim.GeometricFunctions.get_index_min_coord(section1_path, 'x')
        start_index2 = prim.GeometricFunctions.get_index_min_coord(section2_path, 'x')

        reordered_path1 = section1_path[start_index1:] + section1_path[0:start_index1]
        reordered_path2 = section2_path[start_index2:] + section2_path[0:start_index2]

        self.section1 = CrossSection(prim.Path(reordered_path1))
        self.section2 = CrossSection(prim.Path(reordered_path2))


    def get_path(self):
        return self.section1.get_path(), self.section2.get_path()


class CrossSection(object):
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

    def add_hole(self, hole):
        """

        :param CrossSection hole: Hole to add to the cross section.
        """
        if self.holes is None:
            self.holes = list()
        self.holes.append(hole)

    def apply_kerf(self, kerf, max_kerf):
        """
        Applies the kerf value to the two Cross Sections to prepare for cutting with a wire cutter.

        :param float kerf: Generalized Kerf value. Scales based on the length of the cut path (longer path = lower kerf)
         kerf*path_len [mm^2].
        :param float max_kerf: Maximum value allowed for kerf, bandaid for very small sections that explode the kerf.
        """
        if kerf:
            outer_path = self.get_path()
            length_per_point = prim.GeometricFunctions.path_length(outer_path) / len(outer_path)
            kerf_amount_outer = kerf / length_per_point
            kerf_amount_outer = min(kerf_amount_outer, max_kerf)
            for ind, section in enumerate(self._section_list):
                path = section.get_path()
                new_path = prim.GeometricFunctions.offset_curve(path, kerf_amount_outer, 0, 1, add_leading_edge=False)
                new_path = prim.GeometricFunctions.normalize_path_points(new_path, num_points=512)
                if new_path[0] != new_path[-1]:
                    new_path = prim.GeometricFunctions.close_path(new_path)
                self._section_list[ind] = prim.Path(new_path)

            if self.holes:
                for ind, section in enumerate(self.holes):
                    path = section.get_path()
                    length_per_point = prim.GeometricFunctions.path_length(path) / len(path)
                    kerf_amount = kerf / length_per_point
                    kerf_amount = min(kerf_amount, max_kerf)
                    # Holes are offset inwards rather than outwards
                    new_path = prim.GeometricFunctions.offset_curve(path, kerf_amount, 1, 1, add_leading_edge=False)

                    # If the kerf amount results in no valid path, just use the original path instead.
                    if len(new_path) == 0:
                        new_path = path

                    new_path = prim.GeometricFunctions.normalize_path_points(new_path, num_points=512)
                    if new_path[0] != new_path[-1]:
                        new_path = prim.GeometricFunctions.close_path(new_path)
                    self.holes[ind] = prim.Path(new_path)

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
            holes = list()
            for hole in self.holes:
                holes.append(hole.get_path())
            ret_val = holes
        return ret_val

    def get_closest_point_to_hole(self):
        """

        :return: List of tuples where each tuple is the index on the main path and the index on each hole that define
         the two points closest to one another.
        :rtype: list[tuple(int,int)] or None
        """
        ret_val = None
        if self.holes is not None:
            ret_val = list()
            for hole in self.holes:
                min_dist = sys.maxsize
                min_ind = 0
                min_ind_hole = 0
                for ind_hole, point_hole in enumerate(hole.get_path()):
                    for ind, point in enumerate(self.get_path()):
                        dist = np.sqrt((point_hole - point)**2)
                        if dist < min_dist:
                            min_dist = dist
                            min_ind = ind
                            min_ind_hole = ind_hole
                ret_val.append((min_ind, min_ind_hole))
        return ret_val

    def plot(self, color=None, show=True, scatter=False):
        logger = logging.getLogger(__name__)
        prim.GeometricFunctions.plot_path(self.get_path(), color=color, scatter=scatter)
        if self.holes is not None:
            for hole in self.get_path_hole():
                prim.GeometricFunctions.plot_path(hole, color=color, scatter=scatter)

    def plot_gui(self, font_color, scatter_color1, scatter_color2):

        def plot(plot1):
            size = len(self._data)
            x = np.zeros(size)
            y = np.zeros(size)

            for i in range(0, size):
                x[i] = self._data[i]['x']
                y[i] = self._data[i]['y']
            plot1.plot(x, y, font_color)
            plot1.plot(x, y, 'v', markersize=3, color=scatter_color1)
            plot1.axis('equal')

        return plot

    def translate(self, vector):
        PointManip.Transform.translate(self.get_path(), vector=vector)
        if self.holes is not None:
            for hole in self.get_path_hole():
                PointManip.Transform.translate(hole, vector=vector)

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
                                                   prim.Point(point['x'], min_y_p['y'] - 1, 0))

            times_intersected = 0
            for ind in range(0, len(path) - 1):
                point_1 = prim.Point(path[ind]['x'], path[ind]['y'], 0)
                point_2 = prim.Point(path[ind + 1]['x'], path[ind + 1]['y'], 0)
                tmp_line = prim.Line.line_from_points(point_1, point_2)
                # test_line.plot()
                # tmp_line.plot()
                # plt.show()
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
            prim.GeometricFunctions.parallel_curve(prim.GeometricFunctions.normalize_path_points(path, num_points=254),
                                                   thickness, 1, False), path, thickness)
        if path is not None and len(path) > 0:
            path = prim.GeometricFunctions.close_path(path)
            path = prim.GeometricFunctions.normalize_path_points(path, num_points=512)
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

    def is_valid_cut_path(self, tol=0.01, plot=False):
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
                    logger.error('Error: Path of section %s has no points' % (indx + 1))
                    ret_val = False
                if len(self._cut_list_1[indx].get_path()) < 1:
                    logger.error('Error: Path of section %s has no points' % indx)
                    ret_val = False
                if np.sqrt(
                        (self.cut_list_1[indx + 1].get_path()[0] - self._cut_list_1[indx].get_path()[-1]) ** 2) > tol:
                    logger.error('Error: End of a Section is not the start of another (%s != %s)' %
                                 (self.cut_list_1[indx + 1].get_path()[0], self._cut_list_1[indx].get_path()[-1]))
                    if plot:
                        plt.scatter([self.cut_list_1[indx + 1].get_path()[0]['x']],
                                    [self.cut_list_1[indx + 1].get_path()[0]['y']],
                                    c='C9', s=10)
                        plt.scatter([self._cut_list_1[indx].get_path()[-1]['x']],
                                    [self._cut_list_1[indx].get_path()[-1]['y']],
                                    c='C10', s=10)
                    ret_val = False
                if np.sqrt(
                        (self.cut_list_2[indx + 1].get_path()[0] - self._cut_list_2[indx].get_path()[-1]) ** 2) > tol:
                    logger.error('Error: End of a Section is not the start of another (%s != %s)' %
                                 (self.cut_list_2[indx + 1].get_path()[0], self._cut_list_2[indx].get_path()[-1]))
                    if plot:
                        plt.scatter([self.cut_list_2[indx + 1].get_path()[0]['x']],
                                    [self.cut_list_2[indx + 1].get_path()[0]['y']],
                                    c='C11', s=10)
                        plt.scatter([self._cut_list_2[indx].get_path()[-1]['x']],
                                    [self._cut_list_2[indx].get_path()[-1]['y']],
                                    c='C12', s=10)
                    ret_val = False

        return ret_val

    def add_segment_to_cut_lists(self, segment_1, segment_2):
        self._cut_list_1.append(segment_1)
        self._cut_list_2.append(segment_2)

    @staticmethod
    def _add_loopback(cut_path, wing, root_z, tip_z):
        logger = logging.getLogger(__name__)

        offset = prim.Point(wing.start_depth, 0, 0)
        cut_path.add_section_link_from_offset(cut_1_offset=offset, cut_2_offset=offset)

        offset = prim.Point(0, wing.release_height, 0)
        cut_path.add_section_link_from_offset(cut_1_offset=offset, cut_2_offset=offset)

        start1, start2 = cut_path.get_next_start_points()
        next_point1 = prim.Point(0, start1['y'], root_z)
        next_point2 = prim.Point(0, start2['y'], tip_z)
        cut_path.add_section_link_from_abs_coord(point1=next_point1, point2=next_point2, fast_cut=True)

        next_point1 = prim.Point(0, wing.start_height, root_z)
        next_point2 = prim.Point(0, wing.start_height, tip_z)
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
    def create_cut_path_from_wing(wing, wire_cutter, debug_kerf=False, output_dir=None):
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
        if root_foil[0] != root_foil[-1]:
            root_foil = prim.GeometricFunctions.close_path(root_foil)
        if tip_foil[0] != tip_foil[-1]:
            tip_foil = prim.GeometricFunctions.close_path(tip_foil)

        if wing.root_holes is not None:
            holes_root = copy.deepcopy(wing.root_holes)
            holes_tip = copy.deepcopy(wing.tip_holes)

        if wing.root_kerf is not None:
            kerf_root = wing.root_kerf
            kerf_tip = wing.tip_kerf
            if debug_kerf:
                plt.figure()
                prim.GeometricFunctions.plot_path(root_foil, 1, scatter=False)
                prim.GeometricFunctions.plot_path(tip_foil, 2, scatter=False)

            root_foil = prim.GeometricFunctions.offset_curve(root_foil, kerf_root, dir=0, divisions=1,
                                                             add_leading_edge=True)
            tip_foil = prim.GeometricFunctions.offset_curve(tip_foil, kerf_tip, dir=0, divisions=1,
                                                            add_leading_edge=True)

            leading_edge_root = prim.GeometricFunctions.get_point_from_min_coord(root_foil, 'x')
            leading_edge_tip = prim.GeometricFunctions.get_point_from_min_coord(tip_foil, 'x')

            offset = leading_edge_root['x'] if leading_edge_root['x'] < leading_edge_tip['x'] else leading_edge_tip['x']
            if offset < 0:
                PointManip.Transform.translate(root_foil, [-offset, 0, 0])
                if wing.root_holes is not None:
                    for hole in holes_root:
                        PointManip.Transform.translate(hole.get_path(), [-offset, 0, 0])
                PointManip.Transform.translate(tip_foil, [-offset, 0, 0])
                if wing.tip_holes is not None:
                    for hole in holes_tip:
                        PointManip.Transform.translate(hole.get_path(), [-offset, 0, 0])

            if debug_kerf:
                prim.GeometricFunctions.plot_path(root_foil, 3, scatter=False)
                prim.GeometricFunctions.plot_path(tip_foil, 4, scatter=False)

                plt.legend(['Root Foil', 'Tip Foil', 'Root Foil with %smm kerf' % kerf_root,
                            'Tip Foil with %smm kerf' % kerf_tip])
                plt.axis('equal')
                if output_dir is not None:
                    plt.savefig(os.path.join(output_dir, 'kerf_debug.png'))
                else:
                    plt.show()

        root_z = root_foil[0]['z']
        tip_z = tip_foil[0]['z']
        logger.debug('Root: %s | Tip: %s' % (root_foil[0], tip_foil[0]))

        start_point1 = prim.Point(0, 0, root_z)
        start_point2 = prim.Point(0, 0, tip_z)
        next_point1 = start_point1 + prim.Point(0, wing.start_height, 0)
        next_point2 = start_point2 + prim.Point(0, wing.start_height, 0)
        logger.debug('sp1: %s | sp2: %s | np1: %s | np2: %s' % (start_point1, start_point2, next_point1, next_point2))

        seg_link1 = prim.SectionLink(start_point1, next_point1, fast_cut=True)
        seg_link2 = prim.SectionLink(start_point2, next_point2, fast_cut=True)

        cut_path.add_segment_to_cut_lists(segment_1=seg_link1, segment_2=seg_link2)

        start_point1 = next_point1
        start_point2 = next_point2
        next_point1 = start_point1 + prim.Point(wing.start_depth, 0, 0)
        # The start depth of the second axis needs to be offset by the difference between the two foils positioning,
        # this is to account for sweep in the wing

        next_point2 = start_point2 + prim.Point(wing.start_depth - (root_foil[0]['x'] - tip_foil[0]['x']), 0, 0)
        logger.debug('sp1: %s | sp2: %s | np1: %s | np2: %s' % (start_point1, start_point2, next_point1, next_point2))

        test_line = prim.Line.line_from_points(next_point1, next_point2)
        p1 = test_line.get_extrapolated_point(0, constraint_dim='z')
        p2 = test_line.get_extrapolated_point(wire_cutter.wire_length, constraint_dim='z')
        # If the positioning of the leading edges of both airfoils leads to the gantrys going negative in the X or
        # U axis, then offset the cut forwards by the required amount to not go negative.
        if p1['x'] < wing.start_depth or p2['x'] < wing.start_depth:
            offset = max(wing.start_depth - p1['x'], wing.start_depth - p2['x'])
            next_point1 += prim.Point(offset, 0, 0)
            next_point2 += prim.Point(offset, 0, 0)

        # Translate both airfoils by the same offset to keep them inline
        logger.debug('Next Point X: %s', next_point1['x'])
        PointManip.Transform.translate(root_foil, [next_point1['x'], next_point1['y'], 0])
        if wing.root_holes is not None:
            for hole in holes_root:
                PointManip.Transform.translate(hole.get_path(), [next_point1['x'], next_point1['y'], 0])
        PointManip.Transform.translate(tip_foil, [next_point1['x'], next_point1['y'], 0])
        if wing.tip_holes is not None:
            for hole in holes_tip:
                PointManip.Transform.translate(hole.get_path(), [next_point1['x'], next_point1['y'], 0])
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

        plt.figure()
        prim.GeometricFunctions.plot_path(top_root, color='k', scatter=False)
        prim.GeometricFunctions.plot_path(bottom_root, color='r', scatter=False)
        prim.GeometricFunctions.plot_path(top_tip, color='g', scatter=False)
        prim.GeometricFunctions.plot_path(bottom_tip, color='b', scatter=False)
        plt.legend(['top root', 'bottom root', 'top tip', 'bottom tip'])
        if output_dir:
            plt.savefig(os.path.join(output_dir, 'cut_path_debug.png'))
        else:
            plt.show()
        if wing.root_holes is not None:
            holes = holes_root
            holes_tip = holes_tip
            root_hole_order, tip_hole_order = wing.get_closest_point_to_hole()
            hole_order, inds_prim, inds_hole = SpatialPlacement.get_hole_index_order(holes,
                                                                                     root_hole_order)
            hole_order_tip, inds_prim_tip, inds_hole_tip = SpatialPlacement.get_hole_index_order(holes_tip,
                                                                                                 tip_hole_order)
            if 0 < inds_prim[0] <= root_ind_split:
                cut_path.add_segment_to_cut_lists(CrossSection(
                    prim.Path(root_foil[0:inds_prim[0] + 1])),
                    CrossSection(
                        prim.Path(tip_foil[0:inds_prim_tip[0] + 1])))
                last_ind = -1
                for ind in range(0, len(inds_prim)):
                    if 0 < inds_prim[ind] <= root_ind_split:
                        hole_root = holes[hole_order[ind]].get_path()
                        hole_tip = holes_tip[hole_order_tip[ind]].get_path()

                        cut_path.add_segment_to_cut_lists(
                            prim.SectionLink(root_foil[inds_prim[ind]], hole_root[inds_hole[ind]]),
                            prim.SectionLink(tip_foil[inds_prim_tip[ind]],
                                             hole_tip[inds_hole_tip[ind]]))

                        cut_path.add_segment_to_cut_lists(
                            CrossSection(prim.Path(hole_root[inds_hole[ind]:] + hole_root[0:inds_hole[ind] + 1])),
                            CrossSection(
                                prim.Path(hole_tip[inds_hole_tip[ind]:] + hole_tip[0:inds_hole_tip[ind] + 1])))

                        cut_path.add_segment_to_cut_lists(
                            prim.SectionLink(hole_root[inds_hole[ind]], root_foil[inds_prim[ind]]),
                            prim.SectionLink(hole_tip[inds_hole_tip[ind]], tip_foil[inds_prim_tip[ind]]))

                        if ind < len(inds_prim)-1 and 0 <= inds_prim[ind+1] < root_ind_split:
                            cut_path.add_segment_to_cut_lists(
                                CrossSection(prim.Path(root_foil[inds_prim[ind]: inds_prim[ind+1] + 1])),
                                CrossSection(prim.Path(tip_foil[inds_prim_tip[ind]: inds_prim_tip[ind+1] + 1])))
                        last_ind = ind

                cut_path.add_segment_to_cut_lists(
                    CrossSection(prim.Path(root_foil[inds_prim[last_ind]: root_ind_split+1])),
                    CrossSection(prim.Path(tip_foil[inds_prim_tip[last_ind]: tip_ind_split+1])))
            else:
                last_ind = -1
                cut_path.add_segment_to_cut_lists(CrossSection(prim.Path(top_root)), CrossSection(prim.Path(top_tip)))
        else:
            cut_path.add_segment_to_cut_lists(CrossSection(prim.Path(top_root)), CrossSection(prim.Path(top_tip)))

        start_point1 = top_root[-1]
        start_point2 = top_tip[-1]
        logger.debug('TE of Top Root: %s | TE of Top Tip: %s' % (start_point1, start_point2))
        next_point1, next_point2 = CutPath._add_loopback(cut_path, wing, root_z,
                                                         tip_z)

        start_point1 = next_point1
        start_point2 = next_point2
        next_point1 = start_point1 + prim.Point(wing.start_depth, 0, 0)
        next_point2 = start_point2 + prim.Point(wing.start_depth - (root_foil[0]['x'] - tip_foil[0]['x']), 0, 0)
        logger.debug('sp1: %s | sp2: %s | np1: %s | np2: %s' % (start_point1, start_point2, next_point1, next_point2))

        test_line = prim.Line.line_from_points(next_point1, next_point2)
        p1 = test_line.get_extrapolated_point(0, constraint_dim='z')
        p2 = test_line.get_extrapolated_point(wire_cutter.wire_length, constraint_dim='z')

        if p1['x'] < wing.start_depth or p2['x'] < wing.start_depth:
            offset = max(wing.start_depth - p1['x'], wing.start_depth - p2['x'])
            next_point1 += prim.Point(offset, 0, 0)
            next_point2 += prim.Point(offset, 0, 0)

        next_point1 += root_foil[0] - next_point1
        logger.debug('Next Point X after offset: %s', next_point1['x'])
        next_point2 += tip_foil[0] - next_point2
        logger.debug('Root Airfoil Leading Edge: %s', root_foil[0])

        seg_link1 = prim.SectionLink(start_point1, next_point1, fast_cut=False)
        seg_link2 = prim.SectionLink(start_point2, next_point2, fast_cut=False)

        cut_path.add_segment_to_cut_lists(segment_1=seg_link1, segment_2=seg_link2)

        if wing.root_holes is not None and last_ind < len(inds_prim)-1:
            cut_path.add_segment_to_cut_lists(CrossSection(prim.Path(list(reversed(root_foil[inds_prim[-1]:])))),
                                              CrossSection(
                                                  prim.Path(list(reversed(tip_foil[inds_prim_tip[-1]:])))))
            print('debug')
            for ind in reversed(range(last_ind+1, len(inds_prim))):
                hole_root = holes[hole_order[ind]].get_path()
                hole_tip = holes_tip[hole_order_tip[ind]].get_path()

                cut_path.add_segment_to_cut_lists(
                    prim.SectionLink(root_foil[inds_prim[ind]], hole_root[inds_hole[ind]]),
                    prim.SectionLink(tip_foil[inds_prim_tip[ind]],
                                     hole_tip[inds_hole_tip[ind]]))

                cut_path.add_segment_to_cut_lists(
                    CrossSection(prim.Path(hole_root[inds_hole[ind]:] + hole_root[0:inds_hole[ind] + 1])),
                    CrossSection(
                        prim.Path(hole_tip[inds_hole_tip[ind]:] + hole_tip[0:inds_hole_tip[ind] + 1])))

                cut_path.add_segment_to_cut_lists(
                    prim.SectionLink(hole_root[inds_hole[ind]], root_foil[inds_prim[ind]]),
                    prim.SectionLink(hole_tip[inds_hole_tip[ind]], tip_foil[inds_prim_tip[ind]]))

                if root_ind_split < inds_prim[ind-1] < len(root_foil) and len(inds_prim) > 1:
                    cut_path.add_segment_to_cut_lists(
                        CrossSection(prim.Path(list(reversed(root_foil[inds_prim[ind-1]:inds_prim[ind]+1])))),
                        CrossSection(prim.Path(list(reversed(tip_foil[inds_prim_tip[ind-1]:inds_prim_tip[ind]+1])))))
                last_ind = ind

                if len(cut_path.cut_list_1[-1].get_path()) == 0:
                    print('debug')

            cut_path.add_segment_to_cut_lists(
                CrossSection(prim.Path(list(reversed(root_foil[root_ind_split:inds_prim[last_ind]+1])))),
                CrossSection(prim.Path(list(reversed(tip_foil[tip_ind_split:inds_prim_tip[last_ind]+1])))))
        else:
            cut_path.add_segment_to_cut_lists(CrossSection(prim.Path(bottom_root)), CrossSection(prim.Path(bottom_tip)))

        CutPath._add_loopback(cut_path, wing, root_z, tip_z)

        if output_dir is not None:
            cut_path.plot_cut_path(output_dir, False)

        return cut_path

    @staticmethod
    def create_cut_path_from_cross_section_pair_list(cross_section_pairs, work_piece, wire_cutter, section_gap,
                                                     output_dir):
        """

        :param list[CrossSectionPair] cross_section_pairs:
        :return: Returns a list of CutPath[s] created from the cross_section_pairs
        :rtype: list[CutPath]
        """
        logger = logging.getLogger(__name__)
        start = timeit.default_timer()
        sp = SpatialPlacement(work_piece, wire_cutter, vert_spacing=3)
        sp.bin_packing_algorithm(cross_section_pairs, output_dir=output_dir, distance_between_sections=section_gap)
        logger.debug('Finished aligning cross sections on workpiece, took %ss', timeit.default_timer() - start)

        serializer.encode(sp, os.path.join(os.path.dirname(output_dir), 'json'), 'spatial_placement_bf_cp_gen')

        cut_path_1, cut_path_2 = sp.create_section_links_for_cross_section_pairs(method=1)
        serializer.encode(sp, os.path.join(os.path.dirname(output_dir), 'json'), 'spatial_placement_af_cp_gen')
        sp.plot_section_order(output_dir)
        # sp.plot_section_splitting_debug(output_dir)
        cut_paths = list()
        for ind in range(sp.num_sections):
            cut_paths.append(CutPath(cut_path_1[ind], cut_path_2[ind]))
        return cut_paths

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
            plt.plot(x, y, 'r', linewidth=0.6)
            plt.scatter([path[0]['x']], [path[0]['y']], c='C10', s=10)

        for item in self._cut_list_2:
            path = item.get_path()
            len_path = len(path)
            x = np.zeros(len_path)
            y = np.zeros(len_path)
            for ind in range(0, len_path):
                x[ind] = path[ind]['x']
                y[ind] = path[ind]['y']
            plt.plot(x, y, 'k', linewidth=0.6)
            plt.scatter([path[0]['x']], [path[0]['y']], c='C11', s=10)

        self.is_valid_cut_path(plot=True)

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

    def create_cross_section_pairs(self, wall_thickness, origin_plane, spacing, number_sections, output_dir=None,
                                   hollow_section_list=None):
        """

        :param float wall_thickness: Thickness of the part from the outer shell to the inner shell created by a simple
         shell operation.
        :param prim.Plane origin_plane: Plane defining the origin of the slicing process and the normal vector for the
         direction of the cuts.
        :param float spacing: Relative spacing between each sliced cross section.
        :param int number_sections: Number of sections to divide the STL into.
        :param bool open_nose: Whether the nose should be entirly hollow.
        :param bool open_tail: Whether the tail should be entirly hollow.
        :return: List of CrossSectionPairs created from the STL object.
        :rtype: list[CrossSectionPair]
        """
        logger = logging.getLogger(__name__)
        cross_section_pair_list = list()
        self.slice_into_cross_sections(origin_plane, spacing, number_sections, output_dir=output_dir)
        cross_section_list = copy.deepcopy(self.cross_sections)

        for ind in range(1, len(cross_section_list)):
            cross_section_pair_list.append(CrossSectionPair(copy.deepcopy(cross_section_list[ind - 1]),
                                                            copy.deepcopy(cross_section_list[ind])))

        if wall_thickness > 0:
            for ind, hollow in enumerate(hollow_section_list):
                if hollow:
                    cross_section_pair_list[ind].section1.add_simple_hole_from_offset(wall_thickness)
                    cross_section_pair_list[ind].section2.add_simple_hole_from_offset(wall_thickness)

        self._reorganize_holes(cross_section_pair_list)
        self._normalize_number_of_holes(cross_section_pair_list)
        return cross_section_pair_list

    def _reorganize_holes(self, cross_section_pair_list):
        for pair in cross_section_pair_list:
            holes1 = pair.section1.get_path_hole()
            holes2 = pair.section2.get_path_hole()
            # Do no processing if either set of holes is none
            if holes1 is None or holes2 is None:
                continue
            len1 = len(holes1)
            len2 = len(holes2)

            # If there is more than 1 hole, verify that the proper holes algin with one another, if they do not
            # rearrange the hole list.
            if len1 == len2 and len1 > 1:
                # get relative distances for the holes
                distance = list()
                # Todo: This probably only works for a 2 hole case, should test with 2+ and rewrite.
                for hole in holes1:

                    distance.append((hole[0] - holes2[0][0])**2)
                if distance[1] < distance[0]:
                    pair.section2.holes = list(reversed(pair.section2.holes))

    def _normalize_number_of_holes(self, cross_section_pair_list):
        for pair in cross_section_pair_list:
            holes1 = pair.section1.get_path_hole()
            holes2 = pair.section2.get_path_hole()
            # Do no processing if either set of holes is none
            if holes1 is None or holes2 is None:
                continue
            len1 = len(holes1)
            len2 = len(holes2)
            if len1 != len2:
                if len1 > len2:
                    pair.section2.holes = copy.deepcopy(pair.section1.holes)
                else:
                    pair.section1.holes = copy.deepcopy(pair.section2.holes)

    def slice_into_cross_sections(self, origin_plane, spacing, number_sections, output_dir=None):
        """
        Slices the STL into `number_sections` with `spacing` separation normal to the `origin_plane`.

        :param Plane origin_plane: Plane object defining the origin and the direction of the slices.
        :param int number_sections: Number of cross sections to create from the stl.
        :param float spacing: Relative spacing between each cross section in mm.
        """
        logger = logging.getLogger(__name__)
        self.trimesh_cross_sections = list()
        heights = list()
        for i in range(number_sections):
            heights.append(spacing * i)
        origin = np.array(origin_plane.origin)
        normal = np.array(origin_plane.normal)
        sections = self.mesh.section_multiplane(plane_origin=origin, plane_normal=normal, heights=heights)
        if sections is not None:
            self.trimesh_cross_sections.extend(sections)
            self.cross_sections = list()
            for ind1, section in enumerate(sections):
                # Last section could be None if the CrossSection plane is past the length of the stl, skip processing
                # the section if it is None and is the last entry
                if section is None and ind1 == len(sections) - 1:
                    continue
                else:
                    if output_dir:
                        plt.close('all')
                        plt.figure(figsize=(16, 9), dpi=360)
                        section.plot_discrete()
                        plt.axis('equal')
                        plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), 'debug_cross_section_raw_%s.png' % ind1))

                    points = list()
                    # Add the points from trimesh discrete path
                    for ind in range(0, len(section.discrete[0])):
                        points.append(prim.Point(section.discrete[0][ind][0], section.discrete[0][ind][1], 0))
                    # Reorder the points to be in a clockwise order with the first point being at 180 deg from center.
                    if output_dir:
                        plt.close('all')
                        plt.figure(figsize=(16, 9), dpi=360)
                        prim.GeometricFunctions.plot_path(points, color='C1', scatter=True, scatter_color='C2')
                        plt.axis('equal')
                        plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), 'debug_cross_section_b_ar_%s.png' % ind1))

                    path = PointManip.reorder_2d_cw(points, method=5)
                    if output_dir:
                        plt.close('all')
                        plt.figure(figsize=(16, 9), dpi=360)
                        prim.GeometricFunctions.plot_path(path, color='C1', scatter=True, scatter_color='C2')
                        prim.GeometricFunctions.plot_path([path[0]], color='C10', scatter=True, scatter_color='C20')
                        plt.axis('equal')
                        plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), 'debug_cross_section_ar_%s.png' % ind1))

                    # Close the path so that the last point is the first point.
                    path = prim.GeometricFunctions.close_path(path)
                    # Normalize the path so that there are X number points uniform spacing from one another.
                    path = prim.GeometricFunctions.normalize_path_points(path, num_points=512)
                    if output_dir:
                        plt.close('all')
                        plt.figure(figsize=(16, 9), dpi=360)
                        prim.GeometricFunctions.plot_path(path, color='C1', scatter=True, scatter_color='C2')
                        prim.GeometricFunctions.plot_path([path[0]], color='C10', scatter=True, scatter_color='C20')
                        plt.axis('equal')
                        plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), 'debug_cross_section_nor_%s.png' % ind1))

                    path = PointManip.reorder_2d_cw(path, method=6)
                    # Remove any duplicate points that are memory equivalent, these points would get double transformed
                    # later down the line.
                    path = prim.GeometricFunctions.remove_duplicate_memory_from_path(path)
                    if output_dir:
                        plt.close('all')
                        plt.figure(figsize=(16, 9), dpi=360)
                        prim.GeometricFunctions.plot_path(path, color='C1', scatter=True, scatter_color='C2')
                        prim.GeometricFunctions.plot_path([path[0]], color='C10', scatter=True, scatter_color='C20')
                        plt.axis('equal')
                        plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), 'debug_cross_section_f_%s.png' % ind1))

                    # Save off the transformed points as CrossSections
                    cross_section = CrossSection(section_list=[prim.Path(path)])
                    # Holes are present in the geometry
                    if len(section.entities) > 1:
                        for ind in range(1, len(section.entities)):
                            hole_points = list()
                            # Add the points from trimesh discrete path
                            for ind2 in range(0, len(section.discrete[ind])):
                                hole_points.append(
                                    prim.Point(section.discrete[ind][ind2][0], section.discrete[ind][ind2][1], 0))
                            path_hole = PointManip.reorder_2d_cw(hole_points, method=5)
                            path_hole = prim.GeometricFunctions.close_path(path_hole)
                            path_hole = prim.GeometricFunctions.normalize_path_points(path_hole, num_points=512)
                            path_hole = PointManip.reorder_2d_cw(path_hole, method=6)
                            path_hole = prim.GeometricFunctions.remove_duplicate_memory_from_path(path_hole)
                            cross_section.add_hole(CrossSection(section_list=[prim.Path(path_hole)]))
                        longest_length = 0
                        length_ind = 0
                        if longest_length < prim.GeometricFunctions.path_length(cross_section.get_path()):
                            longest_length = prim.GeometricFunctions.path_length(cross_section.get_path())
                        for ind, hole in enumerate(cross_section.holes):
                            if longest_length < prim.GeometricFunctions.path_length(hole.get_path()):
                                longest_length = prim.GeometricFunctions.path_length(hole.get_path())
                                length_ind = ind + 1
                        if length_ind != 0:
                            cross_section_tmp = CrossSection(section_list=cross_section.holes[length_ind-1])
                            cross_section_tmp.add_hole(CrossSection([prim.Path(cross_section.get_path())]))
                            for ind, hole in enumerate(cross_section.holes):
                                if ind != length_ind-1:
                                    cross_section_tmp.add_hole(hole)
                            cross_section = copy.deepcopy(cross_section_tmp)

                    self.cross_sections.append(cross_section)
                    self.logger.debug('section: %s', self.cross_sections[-1])
            # Center the CrossSections so that the center of the most forward cross section is at zero x,y. All
            # CrossSections are translated by the same amount to keep the same relative positioning.
            self.center_cross_sections()
            # self.close_sections()
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


class WingSegment(object):
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

        # Tags used in tandem with gui for menu boxes
        self.root_airfoil_tag = None
        self.tip_airfoil_tag = None
        self.machine_tag = None

        self.root_airfoil = None
        self.tip_airfoil = None

        self.span = None
        self.root_chord = None
        self.tip_chord = None

        self.root_holes = None
        self.tip_holes = None

        self.root_kerf = None
        self.tip_kerf = None

        self.sweep = None
        self.washout = None

        self.align_with_le = False
        self._rotated = False

        self.symmetric = False

        self.prepped = False

        self.start_height = None
        self.start_depth = None
        self.release_height = None

    def prep_for_slicing(self, plot=False, output_dir=None):
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

        # Build the points for any holes that are present
        if self.root_holes is not None:
            for hole in self.root_holes:
                chord_offset = hole.chord_start * self.root_chord
                point_1, point_2 = prim.GeometricFunctions.get_points_with_chord_offset(self.root_airfoil,
                                                                                        chord_offset)
                if point_1['y'] > point_2['y']:
                    point_top = point_1
                    point_bot = point_2
                else:
                    point_top = point_2
                    point_bot = point_1
                # plt.scatter([point_top['x'], point_bot['x']], [point_top['y'], point_bot['y']],color='C5')
                hole.build_points(self.root_chord, point_top, point_bot)

            if self.tip_holes is not None:
                for hole in self.tip_holes:
                    chord_offset = hole.chord_start * self.tip_chord
                    point_1, point_2 = prim.GeometricFunctions.get_points_with_chord_offset(self.tip_airfoil,
                                                                                            chord_offset)
                    if point_1['y'] > point_2['y']:
                        point_top = point_1
                        point_bot = point_2
                    else:
                        point_top = point_2
                        point_bot = point_1
                    # plt.scatter([point_top['x'], point_bot['x']], [point_top['y'], point_bot['y']],color='C6')
                    hole.build_points(self.tip_chord, point_top, point_bot)
            else:
                raise ValueError('Root and Tip of wing must both have a hole if either has a hole.')

        # Rotate the tip airfoil by the washout amount, needs to be done before the airfoil is translated any further
        PointManip.Transform.rotate(self.tip_airfoil, [0, 0, np.deg2rad(self.washout)])
        if self.tip_holes is not None:
            for hole in self.tip_holes:
                PointManip.Transform.rotate(hole.get_path(), [0, 0, np.deg2rad(self.washout)])
        self._rotated = True

        # Translate the tip airfoil back by the sweep amount if the sweep is a positive angle,
        # Translate the root if the sweep is a negative angle
        sweep_offset = self.span * np.sin(np.deg2rad(self.sweep))
        self.logger.info('Offsetting Airfoil by %smm to account for sweep' % sweep_offset)
        PointManip.Transform.translate(self.tip_airfoil, [sweep_offset, 0, 0])
        if self.tip_holes is not None:
            for hole in self.tip_holes:
                PointManip.Transform.translate(hole.get_path(), [sweep_offset, 0, 0])

        # Move the tip airfoil to the end of the span
        PointManip.Transform.translate(self.tip_airfoil, [0, 0, self.span])
        if self.tip_holes is not None:
            for hole in self.tip_holes:
                PointManip.Transform.translate(hole.get_path(), [0, 0, self.span])

        if plot:
            x = list()
            y = list()
            for point in self.root_airfoil:
                x.append(point['x'])
                y.append(point['y'])
            plt.plot(x, y, color='C1')
            if self.root_holes is not None:
                for hole in self.root_holes:
                    path = hole.get_path()
                    prim.GeometricFunctions.plot_path(path, 'C2', False, scatter_color='C11')

            x = list()
            y = list()
            for point in self.tip_airfoil:
                x.append(point['x'])
                y.append(point['y'])
            plt.plot(x, y, color='C3')
            if self.tip_holes is not None:
                for hole in self.tip_holes:
                    path = hole.get_path()
                    prim.GeometricFunctions.plot_path(path, 'C4', False, scatter_color='C12')
            plt.axis('equal')
            if output_dir is not None:
                plt.savefig(os.path.join(output_dir, 'prepped_wing_geom.png'))

        self.prepped = True

    def add_hole_root(self, hole):
        """

        :param list[WingHole] hole: List of points that define the hole pattern to cut.
        :return:
        """
        if self.root_holes is None:
            self.root_holes = list()
        self.root_holes.append(hole)

    def add_hole_tip(self, hole):
        """

        :param list[WingHole] hole: List of points that define the hole pattern to cut.
        :return:
        """
        if self.tip_holes is None:
            self.tip_holes = list()
        self.tip_holes.append(hole)

    def delete_hole(self, curr_index):
        del self.root_holes[curr_index]
        del self.tip_holes[curr_index]

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

            center = wire_cutter.wire_length / 2.0
            wing_half = self.span / 2.0

            if wing_half > center:
                raise AttributeError('Error: Wing does not fit in the wire cutter')

            des_root_pos = center - wing_half
            des_tip_pos = center + wing_half

            print('point tip: %s' % point_tip)
            print('tip destination: %s' % des_tip_pos)
            print('tip offset: %s' % (des_tip_pos - point_tip['z']))
            print('point root: %s' % point_root)
            print('root destination: %s' % des_root_pos)
            print('root offset: %s' % (des_root_pos - point_root['z']))
            PointManip.Transform.translate(self.tip_airfoil, [0, 0, des_tip_pos - point_tip['z']])
            PointManip.Transform.translate(self.root_airfoil, [0, 0, des_root_pos - point_root['z']])
            if self.tip_holes is not None:
                for hole in self.tip_holes:
                    PointManip.Transform.translate(hole.get_path(), [0, 0, des_tip_pos - hole.get_path()[0]['z']])
            if self.root_holes is not None:
                for hole in self.root_holes:
                    PointManip.Transform.translate(hole.get_path(), [0, 0, des_root_pos - hole.get_path()[0]['z']])
            print('')

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

        return np.array(x), np.array(y)

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

    def get_closest_point_to_hole(self):
        """

        :return: Tuple of lists of tuples where each tuple is the index on the main path and the index on each hole that
         define the two points closest to one another. The first list in the top tuple is the root, the second list is
         the tip.
        :rtype: tuple[list[tuple(int,int)]] or None
        """
        ret_val = None
        plt.figure()
        col_ind = 0
        prim.GeometricFunctions.plot_path(self.root_airfoil, color='C%s'%col_ind)
        col_ind += 1
        if self.root_holes is not None:
            ret_val = (list(), list())
            for hole in self.root_holes:
                prim.GeometricFunctions.plot_path(hole.get_path(), color='C%s'%col_ind)
                col_ind += 1
                min_dist = sys.maxsize
                min_ind = 0
                min_ind_hole = 0
                for ind_hole, point_hole in enumerate(hole.get_path()):
                    for ind, point in enumerate(self.root_airfoil):
                        dist = np.sqrt((point_hole - point)**2)
                        if dist < min_dist:
                            min_dist = dist
                            min_ind = ind
                            min_ind_hole = ind_hole
                ret_val[0].append((min_ind, min_ind_hole))
            plt.show()
            for hole in self.tip_holes:
                min_dist = sys.maxsize
                min_ind = 0
                min_ind_hole = 0
                for ind_hole, point_hole in enumerate(hole.get_path()):
                    for ind, point in enumerate(self.tip_airfoil):
                        dist = np.sqrt((point_hole - point)**2)
                        if dist < min_dist:
                            min_dist = dist
                            min_ind = ind
                            min_ind_hole = ind_hole
                ret_val[1].append((min_ind, min_ind_hole))
        return ret_val

    class WingHole(object):
        def __init__(self, chord_start, angle, offset, start_angle, num_points):
            self.points = None
            if start_angle is not None and start_angle < 0:
                start_angle += 360
            self.start_angle = start_angle
            self.chord_start = chord_start
            self.angle = angle
            self.offset = offset
            self.num_points = num_points

        def build_points(self, chord, point_top, point_bottom):
            raise NotImplementedError('build_points is not implemented for class %s' % type(self))

        def get_path(self):
            return self.points

        def finalize_points(self, center):
            PointManip.Transform.rotate(self.points, [0, 0, np.deg2rad(self.angle)], center)
            self.points = prim.GeometricFunctions.close_path(
                prim.GeometricFunctions.normalize_path_points(self.points, self.num_points))

    class RectangularHole(WingHole):
        def __init__(self, chord_start, height, width, angle, offset=0, start_angle=90, num_points=64):
            self.height = height
            self.width = width
            super(WingSegment.RectangularHole, self).__init__(chord_start, angle, offset, start_angle, num_points)

        def build_points(self, chord, point_top, point_bottom):
            self.points = list()

            if 0 <= self.start_angle < 180.0:
                offset = self.offset + self.height/2.0
                y1 = point_bottom['y'] - self.height/2.0 + offset * np.sin(np.deg2rad(self.start_angle))
                y2 = point_bottom['y'] + self.height/2.0 + offset * np.sin(np.deg2rad(self.start_angle))
            else:
                offset = self.offset + self.height/2.0
                y1 = point_top['y'] - self.height/2.0 + offset * np.sin(np.deg2rad(self.start_angle))
                y2 = point_top['y'] + self.height/2.0 + offset * np.sin(np.deg2rad(self.start_angle))
            x1 = chord * self.chord_start - self.width/2.0 + offset * np.cos(np.deg2rad(self.start_angle))
            x2 = chord * self.chord_start + self.width/2.0 + offset * np.cos(np.deg2rad(self.start_angle))
            z = point_bottom['z']
            center = [(x2 + x1)/2.0, (y2+y1)/2.0, z]
            self.points = [prim.Point(x1, y2, z), prim.Point(x2, y2, z), prim.Point(x2, y1, z), prim.Point(x1, y1, z),
                           prim.Point(x1, y2, z)]
            # plt.scatter([center[0]], [center[1]], color='C7')
            self.finalize_points(center)

    class CircleHole(WingHole):
        def __init__(self, chord_start, radius, angle, offset=0, start_angle=90, num_points=128):
            self.radius = radius
            super(WingSegment.CircleHole, self).__init__(chord_start, angle, offset, start_angle, num_points)

        def build_points(self, chord, point_top, point_bottom):
            self.points = list()

            offset = self.offset + self.radius
            if 0 <= self.start_angle < 180.0:
                y1 = point_bottom['y'] + offset * np.sin(np.deg2rad(self.start_angle))
            else:
                y1 = point_top['y'] + offset * np.sin(np.deg2rad(self.start_angle))
            x1 = chord * self.chord_start + offset * np.cos(np.deg2rad(self.start_angle))
            z = point_bottom['z']
            center = [x1, y1, z]
            # plt.scatter([center[0]], [center[1]], color='C7')
            delta = 2*np.pi / (self.num_points-1)
            for ind in range(self.num_points):
                rad = delta * ind
                x = center[0] + np.cos(rad) * self.radius
                y = center[1] + np.sin(rad) * self.radius
                self.points.append(prim.Point(x, y, z))
            self.finalize_points(center)

    class CavityHole(WingHole):
        def __init__(self, chord_start, chord_stop, airfoil_path, thickness, angle=None, offset=None, start_angle=None,
                     num_points=128):
            self.chord_stop = chord_stop
            self.thickness = thickness
            self.airfoil_path = airfoil_path
            super(WingSegment.CavityHole, self).__init__(chord_start, angle, offset, start_angle, num_points)

        def build_points(self, chord, point_top, point_bottom):
            airfoil_path = prim.GeometricFunctions.normalize_path_points(copy.deepcopy(self.airfoil_path), 128)
            PointManip.Transform.scale(airfoil_path, scale=[chord, chord, 1])

            self.points = prim.GeometricFunctions.clean_intersections(
                prim.GeometricFunctions.remove_duplicate_memory_from_path(prim.GeometricFunctions.parallel_curve(
                    airfoil_path, self.thickness, 1, False)), airfoil_path, self.thickness)

            self.points = prim.GeometricFunctions.normalize_path_points(self.points,
                                                                        num_points=int(self.num_points * 1.2))
            if self.points[0] != self.points[-1]:
                self.points = prim.GeometricFunctions.close_path(self.points)
            xmin = chord * self.chord_start
            xmax = chord * self.chord_stop
            new_points = list()
            for point in self.points:
                if xmin <= point['x'] <= xmax:
                    new_points.append(point)
            new_points = prim.GeometricFunctions.close_path(new_points)
            self.points = prim.GeometricFunctions.normalize_path_points(new_points, self.num_points)
