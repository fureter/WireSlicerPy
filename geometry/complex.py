import copy
import logging

import numpy as np
import trimesh as tm
from matplotlib import pyplot as plt

import geometry.primative
from geometry.spatial_manipulation import PointManip
from slicer.wire_cutter import WireCutter
from util import util_functions


class CrossSection():
    """
    Contains a list of sections. Sections are any test_geometry that has a get_path function.
    """
    def __init__(self, section_list):
        self._section_list = section_list

    def __getitem__(self, item):
        if item < len(self._section_list):
            return self._section_list[item]
        else:
            raise ValueError('Error Index of %s out of range of length %s' % (item, len(self._section_list)))

    def add_section(self, section):
        self._section_list.append(section)

    @property
    def section_list(self):
        return self._section_list


class SectionLink():
    """
    Defines connection paths between CrossSections
    """
    def __init__(self, start_point, end_point, cut_speed):
        self.start_point = start_point
        self.end_point = end_point
        self.cut_speed = cut_speed

    def get_movement(self):
        return self.end_point - self.start_point


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
        _, actual_root_chord = geometry.primative.GeometricFunctions.get_point_from_max_coord(self.root_airfoil, 'x')
        if actual_root_chord != self.root_chord:
            scale = self.root_chord / actual_root_chord
            PointManip.Transform.scale(self.root_airfoil, [scale, scale, 1])

        # Check if the tip airfoil chord matches the desired tip chord, if not, scale it to match
        _, actual_tip_chord = geometry.primative.GeometricFunctions.get_point_from_max_coord(self.tip_airfoil, 'x')
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
        print('*'*80)
        print('Point1: %s, Point2: %s' % (self.root_airfoil[0], self.tip_airfoil[0]))
        print('*'*80)

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
        leading_edge_root = geometry.primative.GeometricFunctions.get_point_from_min_coord(self.root_airfoil, 'x')
        PointManip.Transform.translate(self.root_airfoil, [-leading_edge_root['x'], -leading_edge_root['y'],
                                                           -leading_edge_root['z']])

        leading_edge_tip = geometry.primative.GeometricFunctions.get_point_from_min_coord(self.tip_airfoil, 'x')
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
            point_root = geometry.primative.GeometricFunctions.get_point_from_min_coord(self.root_airfoil, 'x')
            point_tip = geometry.primative.GeometricFunctions.get_point_from_min_coord(self.tip_airfoil, 'x')

            center = wire_cutter.wire_length/2
            wing_half = self.span/2

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
        pass

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
        root_thickness = geometry.primative.GeometricFunctions.get_max_thickness(self.root_airfoil, 'y', 'x')
        tip_thickness = geometry.primative.GeometricFunctions.get_max_thickness(self.tip_airfoil, 'y', 'x')

        return root_thickness/self.root_chord, tip_thickness/self.tip_chord
