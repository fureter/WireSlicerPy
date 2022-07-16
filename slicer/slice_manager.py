import logging
import os

import matplotlib.pyplot as plt
import numpy as np
import trimesh.transformations

import g_code.generator as gg
import geometry.complex as comp
import geometry.primative as prim
import slicer.tool_path as tp
import serializer


class SliceManager(object):

    @staticmethod
    def stl_to_gcode(stl_path, name, output_dir, subdivisions, units, wall_thickness, work_piece, wire_cutter,
                     section_gap=5, open_nose=False, open_tail=True, hollow_section_list=None):
        """
        Creates G-Code from an STL file by taking slices of the STL.

        :param str stl_path: Filepath to the STL file to slice.
        :param str name: Name of the project, used for saving files.
        :param str output_dir: Output directory to save files into.
        :param int subdivisions: Number of subdivisions to divide each cross section into.
        :param str units: Units the stl file are saved as, used to convert to mm.
        """
        logger = logging.getLogger(__name__)
        SliceManager.create_folders(output_dir, name)
        output_dir = os.path.join(output_dir, name)
        cut_stl = comp.STL(file_path=stl_path, units=units)
        cut_stl.mesh.convert_units('mm')
        # rot_mat = trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0], [0, 0, 0])
        # cut_stl.mesh.apply_transform(rot_mat)
        bounding_box = cut_stl.mesh.bounds
        logger.info('Bounds: %s' % bounding_box)

        x_len = abs(bounding_box[1][0] - bounding_box[0][0])
        y_len = abs(bounding_box[1][1] - bounding_box[0][1])
        z_len = abs(bounding_box[1][2] - bounding_box[0][2])
        max_len = max(x_len, y_len, z_len)

        x_norm = 0
        y_norm = 0
        z_norm = 0

        if max_len == x_len:
            used_index = 0
            x_norm = 1
        if max_len == y_len:
            used_index = 1
            y_norm = 1
        if max_len == z_len:
            used_index = 2
            z_norm = 1

        normal = np.array([x_norm, y_norm, z_norm])
        scale = 0.999 if bounding_box[0][used_index] < 0 else 1.001
        extent = bounding_box[0] * normal * scale
        slice_plane = prim.Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])
        spacing = work_piece.thickness
        length = int(abs(bounding_box[1][used_index] - bounding_box[0][used_index]) / spacing) + 1

        if hollow_section_list is None:
            hollow_section_list = [True] * (length-1)
            if not open_nose:
                hollow_section_list[0] = False
            if not open_tail:
                hollow_section_list[-1] = False
        else:
            hollow_section_list_tmp = [False] * (length-1)
            for index in hollow_section_list:
                hollow_section_list_tmp[index] = True
            hollow_section_list = hollow_section_list_tmp

        section_list = cut_stl.create_cross_section_pairs(wall_thickness=wall_thickness, origin_plane=slice_plane,
                                                          spacing=spacing, number_sections=length,
                                                          hollow_section_list=hollow_section_list)
        # section_list = section_list[1:]  # todo remove

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

        subdivided_list = comp.CrossSectionPair.subdivide_list(subdivisions, section_list)
        for ind, section in enumerate(subdivided_list):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=320)
            prim.GeometricFunctions.plot_path(section.section1.get_path(), color='C1', scatter=False)
            prim.GeometricFunctions.plot_path(section.section2.get_path(), color='C2', scatter=False)
            section.apply_kerf(kerf=wire_cutter.kerf, max_kerf=wire_cutter.max_kerf)
            # section.align_start()
            prim.GeometricFunctions.plot_path(section.section1.get_path(), color='C3', scatter=True)
            prim.GeometricFunctions.plot_path(section.section2.get_path(), color='C4', scatter=True)
            prim.GeometricFunctions.plot_path([section.section1.get_path()[0]], color='C10', scatter=True)
            prim.GeometricFunctions.plot_path([section.section2.get_path()[0]], color='C11', scatter=True)

            plt.legend(['Section 1 orig', 'Section 2 orig', 'Section 1 kerf', 'Section 2 kerf'])
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_subdivisions_%s.png' % (name, ind)))
            plt.show()

        serializer.encode(subdivided_list, output_dir=os.path.join(output_dir, 'json'),
                          file_name='subdivided_section_list')

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
                                        part=ind+1)

            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=400)
            tool_path.plot_tool_paths()
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_final_tool_path_%s.png' % (name, ind)))
            plt.show()

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
        gcode.create_relative_gcode(output_dir=output_dir, name=name, tool_path=tool_path,
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
