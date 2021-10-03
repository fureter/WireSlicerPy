import logging
import os

import matplotlib.pyplot as plt
import numpy as np

import g_code.generator as gg
import geometry.complex as comp
import geometry.primative as prim
import slicer.tool_path as tp


class SliceManager(object):
    """

    :param work_piece: Holds the dimensions for the material stock that is being cut.
    :type work_piece: prim.WorkPiece
    """

    def __init__(self, wire_cutter, work_piece):
        self.wire_cutter = wire_cutter
        self.work_piece = work_piece

    def stl_to_gcode(self, stl_path, name, output_dir, subdivisions, units, wall_thickness):
        """
        Creates G-Code from an STL file by taking slices of the STL.

        :param str stl_path: Filepath to the STL file to slice.
        :param str name: Name of the project, used for saving files.
        :param str output_dir: Output directory to save files into.
        :param int subdivisions: Number of subdivisions to divide each cross section into.
        :param str units: Units the stl file are saved as, used to convert to mm.
        """
        logger = logging.getLogger(__name__)
        self.create_folders(output_dir)
        cut_stl = comp.STL(file_path=stl_path, units=units)
        cut_stl.mesh.convert_units('mm')
        bounding_box = cut_stl.mesh.bounds
        logger.info('Bounds: %s' % bounding_box)

        normal = np.array([1, 0, 0])
        scale = 0.999 if bounding_box[0][1] < 0 else 1.001
        extent = bounding_box[0] * normal * scale
        slice_plane = prim.Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])
        spacing = self.work_piece.thickness
        length = int(abs(bounding_box[1][0] - bounding_box[0][0]) / spacing) + 1
        section_list = cut_stl.create_cross_section_pairs(wall_thickness=wall_thickness, origin_plane=slice_plane,
                                                          spacing=spacing, number_sections=length,
                                                          open_nose=False, open_tail=True)

        for ind, section in enumerate(section_list):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=320)
            section.plot_subdivide_debug(subdivisions, radius=100)
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_Cross_Section_orig_%s.png' % (name, ind)))
            plt.show()

        subdivided_list = comp.CrossSectionPair.subdivide_list(subdivisions, section_list)
        for ind, section in enumerate(subdivided_list):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=320)
            section.plot_subdivide_debug(3, radius=40)
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_subdivisions_%s.png' % (name, ind)))
            plt.show()

        cut_paths = comp.CutPath.create_cut_path_from_cross_section_pair_list(subdivided_list, self.work_piece,
                                                                              section_gap=10,
                                                                              wire_cutter=self.wire_cutter,
                                                                              output_dir=os.path.join(output_dir,
                                                                                                      'plots'))
        for ind, cut_path in enumerate(cut_paths):
            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=360)
            cut_path.plot_cut_path()
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_final_cut_path_%s.png' % (name, ind)))
            plt.show()

            tool_path = tp.ToolPath.create_tool_path_from_cut_path(cut_path=cut_path, wire_cutter=self.wire_cutter)
            tool_path.zero_forwards_path_for_cutting()
            gcode = gg.GCodeGenerator(self.wire_cutter, travel_type=gg.TravelType.CONSTANT_RATIO)
            gcode.create_relative_gcode(file_path=os.path.join(output_dir, '%s_gcode_p%s.txt' % (name, ind + 1)),
                                        tool_path=tool_path)

            plt.close('all')
            plt.figure(figsize=(16, 9), dpi=400)
            tool_path.plot_tool_paths()
            plt.axis('equal')
            plt.savefig(os.path.join(os.path.join(output_dir, 'plots'), '%s_final_tool_path_%s.png' % (name, ind)))
            plt.show()

    def wing_to_gcode(self, wing, output_dir):
        self.create_folders(output_dir)
        pass

    def create_folders(self, output_dir):
        if not os.path.exists(os.path.join(output_dir, 'plots')):
            os.mkdir(os.path.join(output_dir, 'plots'))
