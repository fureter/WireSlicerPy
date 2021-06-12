import copy
import logging

import numpy as np
import trimesh as tm
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import geometry.primative as prim
from geometry.spatial_manipulation import PointManip
from slicer.wire_cutter import WireCutter
from util import util_functions


class CrossSection():
    """
    Contains a list of sections. Sections are any class that defines a get_path function. Valid CrossSections must
    contain at least one section, and the end of one section must be the start of the next section.
    """

    def __init__(self, section_list):
        if not isinstance(section_list, list):
            section_list = [section_list]
        self._section_list = section_list

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
        if len(self._cut_list_1) != len(self._cut_list_1):
            logger.error('Error: cut_list lengths do not match')
            ret_val = False
        else:
            for indx in range(0, len(self._cut_list_1) - 1):
                if self.cut_list_1[indx + 1].get_path()[0] != self._cut_list_1[indx].get_path()[-1]:
                    logger.error('Error: End of a Section is the the start of another (%s != %s)' %
                                 (self.cut_list_1[indx + 1].get_path()[0], self._cut_list_1[indx].get_path()[-1]))
                    ret_val = False
                if self.cut_list_2[indx + 1].get_path()[0] != self._cut_list_2[indx].get_path()[-1]:
                    logger.error('Error: End of a Section is the the start of another (%s != %s)' %
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
        cut_path.add_section_link_from_abs_coord(point1=next_point1, point2=next_point2)

        next_point1 = prim.Point(0, wire_cutter.start_height, root_z)
        next_point2 = prim.Point(0, wire_cutter.start_height, tip_z)
        cut_path.add_section_link_from_abs_coord(point1=next_point1, point2=next_point2)

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
    def create_cut_path_from_wing(wing, wire_cutter):
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
        root_z = wing.root_airfoil[0]['z']
        tip_z = wing.tip_airfoil[0]['z']
        logger.debug('Root Z Coord: %s | Tip Z Coord: %s' % (root_z, tip_z))

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

        seg_link1 = prim.SectionLink(start_point1, next_point1, fast_cut=False)
        seg_link2 = prim.SectionLink(start_point2, next_point2, fast_cut=False)

        cut_path.add_segment_to_cut_lists(segment_1=seg_link1, segment_2=seg_link2)

        # Translate both airfoils by the same offset to keep them inline
        PointManip.Transform.translate(root_foil, [next_point1['x'], next_point1['y'], 0])
        PointManip.Transform.translate(tip_foil, [next_point1['x'], next_point1['y'], 0])

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

        seg_link1 = prim.SectionLink(start_point1, next_point1, fast_cut=False)
        seg_link2 = prim.SectionLink(start_point2, next_point2, fast_cut=False)

        cut_path.add_segment_to_cut_lists(segment_1=seg_link1, segment_2=seg_link2)

        cut_path.add_segment_to_cut_lists(CrossSection(prim.Path(bottom_root)), CrossSection(prim.Path(bottom_tip)))

        CutPath._add_loopback(cut_path, wire_cutter, root_z, tip_z)

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

    def plot_cut_path(self):
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
        plt.show()

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

    """

    def __init__(self, file_path, logger=None, units='cm'):
        """

        :param file_path:
        """
        if logger is None:
            logger = logging.getLogger()
        self.logger = logger
        self.cross_sections = None
        self._file_path = file_path
        self._setup(units=units)

    def slice_into_cross_sections(self, origin_plane, spacing):
        """

        :param Plane origin_plane:
        :param float spacing:
        :return:
        """
        self.cross_sections = list()
        origin = np.array(origin_plane.origin)
        normal = np.array(origin_plane.normal)
        section = self.mesh.section(plane_origin=origin, plane_normal=normal)
        if section is not None:
            self.cross_sections.append(section)
            next_origin = origin + normal * spacing
            while section is not None:
                section = self.mesh.section(plane_origin=next_origin, plane_normal=normal)
                next_origin = next_origin + normal * spacing
                if section is not None:
                    self.cross_sections.append(section)

            next_origin = origin - normal * spacing
            section = self.mesh.section(plane_origin=next_origin, plane_normal=normal)
            if section is not None:
                self.cross_sections.append(section)
                next_origin = origin + normal * spacing
                while section is not None:
                    section = self.mesh.section(plane_origin=next_origin, plane_normal=normal)
                    next_origin = next_origin - normal * spacing
                    if section is not None:
                        self.cross_sections.append(section)
        else:
            raise AttributeError('Error: Plane with origin(%s) does not intersect the STL' % origin_plane.origin)

    def plot_stl(self):
        self.mesh.show()

    def plot_cross_sections(self, bounds):
        if self.cross_sections is None:
            raise AttributeError('Error: Cross sections have not been generated for this STL')
        num_sections = len(self.cross_sections)
        self.logger.info('Number of sections being plotted: %s' % num_sections)
        (r, c) = util_functions.get_r_and_c_from_num(num_sections)
        self.logger.info('Creating section subplots with r: %s and c: %s' % (r, c))
        i = 1
        for section in self.cross_sections:
            ax = plt.subplot(int(r), int(c), i)
            section.to_planar()[0].plot_entities()
            ax.set_title('Cross Section: %s' % i)
            ax.set_xlim([bounds[0][1], bounds[1][1]])
            ax.set_ylim([bounds[0][2], bounds[1][2]])
            i += 1
            plt.show(block=False)

    def _setup(self, units):
        tm.util.attach_to_log(level=logging.INFO)
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
        plt.show()
