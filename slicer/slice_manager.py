import logging
import os

import matplotlib.pyplot as plt
import numpy as np
import trimesh

import g_code.generator as gg
import geometry.complex as comp
import geometry.primative as prim
import serializer
import slicer.tool_path as tp


class SliceManager(object):

    @staticmethod
    def stl_to_gcode(stl_path, name, output_dir, subdivisions, units, wall_thickness, work_piece, wire_cutter,
                     section_gap=5, open_nose=False, open_tail=True, hollow_section_list=None, flip_axis=None,
                     slice_axis=0, num_points=512):
        """
        Creates G-Code from an STL file by taking slices of the STL.

        :param str stl_path: Filepath to the STL file to slice.
        :param str name: Name of the project, used for saving files.
        :param str output_dir: Output directory to save files into.
        :param int subdivisions: Number of subdivisions to divide each cross section into.
        :param str units: Units the stl file are saved as, used to convert to mm.
        """
        logger = logging.getLogger(__name__)

        subdivided_list = SliceManager.stl_precondition(stl_path=stl_path, logger=logger, units=units,
                                                        output_dir=output_dir, work_piece=work_piece,
                                                        hollow_section_list=hollow_section_list,
                                                        open_nose=open_nose, open_tail=open_tail,
                                                        wall_thickness=wall_thickness,
                                                        name=name, subdivisions=subdivisions, wire_cutter=wire_cutter,
                                                        flip_axis=flip_axis, slice_axis=slice_axis,
                                                        num_points=num_points)

        output_dir = os.path.join(output_dir, name)
        cut_paths = comp.CutPath.create_cut_path_from_cross_section_pair_list(subdivided_list, work_piece,
                                                                              section_gap=section_gap,
                                                                              wire_cutter=wire_cutter,
                                                                              output_dir=os.path.join(output_dir,
                                                                                                      'plots'))
        serializer.encode(cut_paths, output_dir=os.path.join(output_dir, 'json'),
                          file_name='cut_paths')

        for ind, cut_path in enumerate(cut_paths):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=360)
            cut_path.plot_cut_path()
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_final_cut_path_%s.png' % (name, ind)))
            plt.show()

            tool_path = tp.ToolPath.create_tool_path_from_cut_path(cut_path=cut_path, wire_cutter=wire_cutter)
            tool_path.zero_forwards_path_for_cutting()
            gcode = gg.GCodeGenerator(wire_cutter, travel_type=gg.TravelType.CONSTANT_RATIO)
            gcode.create_relative_gcode(output_dir=output_dir, name=name, tool_path=tool_path, wire_cutter=wire_cutter,
                                        part=ind + 1)

            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=400)
            tool_path.plot_tool_paths()
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_final_tool_path_%s.png' % (name, ind)))
            plt.show()

        return subdivided_list

    @staticmethod
    def stl_precondition(stl_path, logger, units, output_dir, work_piece, hollow_section_list, open_nose, open_tail,
                         wall_thickness, name, subdivisions, wire_cutter, flip_axis=False, slice_axis=0,
                         num_points=512):
        SliceManager.create_folders(output_dir, name)
        output_dir = os.path.join(output_dir, name)
        cut_stl = comp.STL(file_path=stl_path, units=units)
        cut_stl.mesh.convert_units('mm')
        bounding_box = cut_stl.mesh.bounds
        logger.info('Bounds: %s' % bounding_box)

        x_len = abs(bounding_box[1][0] - bounding_box[0][0])
        y_len = abs(bounding_box[1][1] - bounding_box[0][1])
        z_len = abs(bounding_box[1][2] - bounding_box[0][2])
        lengths = sorted([x_len, y_len, z_len])

        used_index = 0
        x_norm = 0
        y_norm = 0
        z_norm = 0
        flip_rot = [0, 0, 0]

        if (lengths[2] == x_len and slice_axis == 0) or (lengths[1] == x_len and slice_axis == 1) or (
                lengths[0] == x_len and slice_axis == 2):
            used_index = 0
            x_norm = 1
            flip_rot = [0, 1, 0]
        if (lengths[2] == y_len and slice_axis == 0) or (lengths[1] == y_len and slice_axis == 1) or (
                lengths[0] == y_len and slice_axis == 2):
            used_index = 1
            y_norm = 1
            flip_rot = [0, 0, 1]
        if (lengths[2] == z_len and slice_axis == 0) or (lengths[1] == z_len and slice_axis == 1) or (
                lengths[0] == z_len and slice_axis == 2):
            used_index = 2
            z_norm = 1
            flip_rot = [1, 0, 0]

        normal = np.array([x_norm, y_norm, z_norm])
        scale = 0.999 if bounding_box[0][used_index] < 0 else 1.001
        extent = bounding_box[0] * normal * scale
        slice_plane = prim.Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])
        spacing = work_piece.thickness
        length = int(abs(bounding_box[1][used_index] - bounding_box[0][used_index]) / spacing) + 1

        e_msg = 'Error: Index specified for hollow list is greater than number of cross sections'

        if hollow_section_list is None:
            hollow_section_list = [True] * (length - 1)
            if not open_nose:
                hollow_section_list[0] = False
            if not open_tail:
                hollow_section_list[-1] = False
        else:
            hollow_section_list_tmp = [False] * (length - 1)
            for index in hollow_section_list:
                assert index < len(hollow_section_list_tmp), e_msg
                hollow_section_list_tmp[index] = True
            hollow_section_list = hollow_section_list_tmp

        if flip_axis:
            rot_mat = trimesh.transformations.rotation_matrix(np.pi, flip_rot, [0, 0, 0])
            cut_stl.mesh.apply_transform(rot_mat)
            bounding_box = cut_stl.mesh.bounds
            scale = 0.999 if bounding_box[0][used_index] < 0 else 1.001
            extent = bounding_box[0] * normal * scale
            slice_plane = prim.Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])

        section_list = cut_stl.create_cross_section_pairs(wall_thickness=wall_thickness, origin_plane=slice_plane,
                                                          spacing=spacing, number_sections=length,
                                                          hollow_section_list=hollow_section_list,
                                                          num_points=num_points)

        serializer.encode(section_list, output_dir=os.path.join(output_dir, 'json'), file_name='cross_section_list')

        for ind, section in enumerate(section_list):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=320)
            prim.GeometricFunctions.plot_path(section.section1.get_path(), color='C1', scatter=False)
            prim.GeometricFunctions.plot_path(section.section2.get_path(), color='C2', scatter=False)
            if section.section1.get_path_hole() is not None:
                for hole in section.section1.get_path_hole():
                    prim.GeometricFunctions.plot_path(hole, color='C3', scatter=False)
            if section.section2.get_path_hole() is not None:
                for hole in section.section2.get_path_hole():
                    prim.GeometricFunctions.plot_path(hole, color='C4', scatter=False)
            prim.GeometricFunctions.plot_path([section.section1.get_path()[0]], color='C10', scatter=True)
            prim.GeometricFunctions.plot_path([section.section2.get_path()[0]], color='C11', scatter=True)
            plt.legend(['Section 1 outer', 'Section 2 outer', 'Section 1 inner', 'Section 2 inner'])
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_Cross_Section_orig_%s.png' % (name, ind)))
            plt.show()

        subdivided_list = comp.CrossSectionPair.subdivide_list(subdivisions, section_list, num_points=num_points)
        for ind, section in enumerate(subdivided_list):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=320)
            prim.GeometricFunctions.plot_path(section.section1.get_path(), color='C1', scatter=False)
            prim.GeometricFunctions.plot_path(section.section2.get_path(), color='C2', scatter=False)
            section.apply_kerf(kerf=wire_cutter.kerf, max_kerf=wire_cutter.max_kerf)
            # section.align_start()
            prim.GeometricFunctions.plot_path(section.section1.get_path(), color='C3', scatter=True)
            prim.GeometricFunctions.plot_path(section.section2.get_path(), color='C4', scatter=True)
            prim.GeometricFunctions.plot_path([section.section1.get_path()[0]], color='C10', scatter=True,
                                              scatter_size=40)
            prim.GeometricFunctions.plot_path([section.section2.get_path()[0]], color='C11', scatter=True,
                                              scatter_size=40)
            if section.section1.get_path_hole() is not None:
                for hole in section.section1.get_path_hole():
                    prim.GeometricFunctions.plot_path(hole, color='C3', scatter=False)
            if section.section2.get_path_hole() is not None:
                for hole in section.section2.get_path_hole():
                    prim.GeometricFunctions.plot_path(hole, color='C4', scatter=False)

            plt.legend(['Section 1 orig', 'Section 2 orig', 'Section 1 kerf', 'Section 2 kerf'])
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_subdivisions_%s.png' % (name, ind)))
            plt.show()

        serializer.encode(subdivided_list, output_dir=os.path.join(output_dir, 'json'),
                          file_name='subdivided_section_list')
        return subdivided_list

    @staticmethod
    def wing_to_gcode(wing, wire_cutter, output_dir):
        SliceManager.create_folders(output_dir, wing.name)
        plot_dir = os.path.join(output_dir, wing.name, 'plots')

        gcode = gg.GCodeGenerator(wire_cutter, travel_type=gg.TravelType.CONSTANT_RATIO)

        output_dir = os.path.join(output_dir, wing.name)

        wing.prep_for_slicing(plot=True, output_dir=plot_dir)
        wing.center_to_wire_cutter(wire_cutter=wire_cutter)
        cut_path = comp.CutPath.create_cut_path_from_wing(wing, wire_cutter=wire_cutter, debug_kerf=True,
                                                          output_dir=plot_dir)
        tool_path = tp.ToolPath.create_tool_path_from_cut_path(cut_path=cut_path, wire_cutter=wire_cutter,
                                                               output_dir=plot_dir)
        tool_path.zero_forwards_path_for_cutting()
        name = '%s_left.txt' if wing.symmetric else '%s.txt'
        gcode.create_relative_gcode(output_dir=output_dir, name=name % wing.name, tool_path=tool_path,
                                    wire_cutter=wire_cutter)

        if wing.symmetric:
            wing.flip_tip_and_root()
            cut_path = comp.CutPath.create_cut_path_from_wing(wing, wire_cutter)
            tool_path = tp.ToolPath.create_tool_path_from_cut_path(cut_path, wire_cutter=wire_cutter)
            tool_path.zero_forwards_path_for_cutting()
            gcode.create_relative_gcode(output_dir=output_dir, name='%s_right.txt' % wing.name,
                                        tool_path=tool_path,
                                        wire_cutter=wire_cutter)

    @staticmethod
    def create_folders(output_dir, name):
        if not os.path.exists(os.path.join(output_dir, name)):
            os.mkdir(os.path.join(output_dir, name))
        if not os.path.exists(os.path.join(output_dir, name, 'plots')):
            os.mkdir(os.path.join(output_dir, name, 'plots'))
        if not os.path.exists(os.path.join(output_dir, name, 'json')):
            os.mkdir(os.path.join(output_dir, name, 'json'))


class CadParts(object):
    """
    Class to track required configurations for slicing CAD parts. This class is used for interfacing between the CAD
    GUI window and the SliceManager. This class is also used for saving CADPart settings across different GUI
    selections. This differs from wings and wire_cutters as they are completely self contained object and therefore do
    not require an intermediary like this.
    """

    def __init__(self, name, logger):
        self.name = name
        self.logger = logger

        self.stl_tag = None
        self.units = None
        self.machine_tag = None

        self.wp_length = None
        self.wp_height = None
        self.wp_thickness = None
        self.subdivisions = None
        self.wall_thickness = None
        self.section_gap = None

        self.num_points = None
        self.kerf = None
        self.max_kerf = None

        self.slice_axis = None
        self.flip_axis = None

        self.open_nose = None
        self.open_tail = None
        self.hollow_list = None
