import logging
import os.path
import sys
import copy

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import trimesh.transformations

import util.util_functions
import wire_slicer
from slicer.wire_cutter import WireCutter
from slicer.tool_path import ToolPath
from geometry.parser import Dat
from geometry.primative import Point
from geometry.primative import Line
from geometry.primative import Spline
from geometry.primative import Plane
from geometry.primative import GeometricFunctions
from geometry.primative import WorkPiece
from geometry.complex import STL
from geometry.complex import CrossSectionPair
from geometry.complex import CutPath
from geometry.complex import WingSegment
from geometry.spatial_manipulation import PointManip
from g_code.generator import GCodeGenerator
from g_code.generator import CutLayout
from g_code.generator import TravelType


def main():
    wire_slicer.setup(logger_level=logging.DEBUG)

    matplotlib.use('SVG')

    logger = logging.getLogger(__name__)

    # file_path = r'assets/Airfoils/MH70.dat'
    # naca0009_file_path = r'assets/Airfoils/naca0009.dat'
    #
    # ag35_file_path = r'assets/Airfoils/ag35.dat'
    # s5020_file_path = r'assets/Airfoils/s5020.dat'
    #
    # naca0009_data = Dat(filepath=naca0009_file_path)
    # #ag35_data.plot_points_2d()
    #
    # naca0009_reord = Dat(data=PointManip.reorder_2d_cw(copy.deepcopy(naca0009_data.get_data()), method=2))
    # #ag35_reord.plot_points_2d()
    #
    # naca0009_path = Spline(naca0009_reord.get_data(), resolution=4).get_path()
    #
    # logger.info('Profile Name: %s' % naca0009_data.name)
    #
    # wing = WingSegment(name=naca0009_data.name, logger=logger)
    # wing.set_span(220)
    # wing.set_root_chord(150)
    # wing.set_tip_chord(80)
    # wing.set_root_airfoil(copy.deepcopy(naca0009_path))
    # wing.set_tip_airfoil(copy.deepcopy(naca0009_path))
    # wing.set_washout(0)
    # wing.set_sweep(12)
    #
    #
    wire_len = 200
    wire_cutter = WireCutter(wire_length=wire_len, max_height=300.0, max_speed=100.0, min_speed=0.01,
                             release_height=100.0,
                             start_height=25.0, start_depth=20.0)
    wire_cutter.set_kerf(kerf=1.0)

    wire_cutter.set_gcode_statup(g_code=['G17', 'G21'])
    #
    # wing.prep_for_slicing(plot=True)
    # plt.show()
    # wing.center_to_wire_cutter(wire_cutter=wire_cutter)
    # wing.plot_cut_planform()
    # #wing.align_leading_edge_with_wire()
    # wing.plot_cut_planform()
    # plt.legend(['Original Wing', 'Wing aligned to Wire'])
    # plt.axis('equal')
    # plt.show()
    # print('Root Airfoil Point after prepping: %s' % wing.root_airfoil[0])
    #
    # cut_path = CutPath.create_cut_path_from_wing(wing, wire_cutter=wire_cutter, debug_kerf=True)
    # cut_path.plot_section_link_connections()
    # cut_path.plot_cut_path()
    #
    # tool_path = ToolPath.create_tool_path_from_cut_path(cut_path=cut_path, wire_cutter=wire_cutter)
    # tool_path.zero_forwards_path_for_cutting()
    # tool_path.plot_tool_paths()
    # tool_path.plot_tool_path_connections(step=3)
    # plt.show()
    # # Animation runs endlessly when trying to save, might be an issue with my ffmpeg install
    # #tool_path.animate(file_path='./debug/%s.mp4', title='Naca0009_horz_stab_test')
    # tool_path.animate()
    #
    # gcode = GCodeGenerator(wire_cutter, travel_type=TravelType.CONSTANT_RATIO)
    # gcode.create_relative_gcode(file_path=r'./assets/GCode/%s_test.txt' % wing.name, tool_path=tool_path)
    # #
    # plt.axis('equal')
    # plt.show()
    #
    # wing.flip_tip_and_root()
    # cut_path = CutPath.create_cut_path_from_wing(wing, wire_cutter)
    # tool_path = ToolPath.create_tool_path_from_cut_path(cut_path, wire_cutter=wire_cutter)
    # tool_path.zero_forwards_path_for_cutting()
    # gcode.create_relative_gcode(file_path=r'./assets/GCode/%s_right_test.txt' % wing.name, tool_path=tool_path)
    # plt.axis('equal')
    # plt.show()

    # plt.legend(['goe430 @ %smm' % profile_2_dist, 'naca0009 @ %smm' % profile_1_dist, 'tool_path XY @ %smm' % wire_len,
    #             'tool_path UZ @ 0mm'])
    # plt.axis('equal')
    # plt.show()
    output_dir = os.path.join(os.path.dirname(__file__), 'plots', 'Impulse_Reaction')

    test_stl = STL(file_path=r'./assets/STLs/Fuselage_jet.stl', units='mm')
    test_stl.mesh.convert_units('mm')
    #test_stl.plot_stl()
    normal = np.array([1, 0, 0])
    # test_stl.mesh.apply_transform(trimesh.transformations.scale_matrix(factor=2))
    bounding_box = test_stl.mesh.bounds
    logger.info('Bounds: %s' % bounding_box)
    scale = 0.999 if bounding_box[0][1] < 0 else 1.001
    extent = bounding_box[0] * normal*scale
    slice_plane = Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])
    spacing = 51
    length = int(abs(bounding_box[1][0] - bounding_box[0][0]) / spacing) + 1
    section_list = test_stl.create_cross_section_pairs(wall_thickness=12.0, origin_plane=slice_plane, spacing=spacing,
                                                       length=length, open_nose=False, open_tail=True)
    for ind, section in enumerate(section_list):
        plt.close('all')
        plt.figure(figsize=(16, 9), dpi=320)
        section.plot_subdivide_debug(3, radius=100)
        plt.axis('equal')
        plt.savefig(os.path.join(output_dir, 'Cross_Section_orig_%s.png' % ind))
        plt.show()
    # r, c = util.util_functions.get_r_and_c_from_num(len(section_list))
    # for ind in range(len(section_list)):
    #     plt.subplot(r, c, ind + 1)
    #     section_list[ind].plot_subdivide_debug(0, 0)
    #     plt.axis('equal')
    # plt.show()

    subdivided_list = CrossSectionPair.subdivide_list(4, section_list)
    for ind, section in enumerate(subdivided_list):
        plt.close('all')
        plt.figure(figsize=(16, 9), dpi=320)
        section.plot_subdivide_debug(3, radius=40)
        plt.axis('equal')
        plt.savefig(os.path.join(output_dir, 'Cross_Section_subdivide_%s.png' % ind))
        plt.show()
    # subdivided_list = subdivided_list[0:10]

    # not all subdivision are coming out clock_wise, subdivision at index 27 is counter clockwise
    # ind = 1
    # issue_sdl = subdivided_list[2]
    # GeometricFunctions.plot_path(issue_sdl.get_path()[ind], None)
    # GeometricFunctions.parallel_curve(issue_sdl.get_path()[ind], offset_scale=8, dir=0, plot_debug=True)
    # para = GeometricFunctions.offset_curve(issue_sdl.get_path()[ind], offset_scale=8, dir=0, divisions=1,
    #                                        add_leading_edge=False)
    # GeometricFunctions.plot_path(para, None)
    # plt.show()

    work_piece = WorkPiece(width=1000, height=600, thickness=spacing)
    # GeometricFunctions.center_path(issue_sdl.get_path()[0])
    test_path = CutPath.create_cut_path_from_cross_section_pair_list(subdivided_list, work_piece, wire_cutter,
                                                                     output_dir=output_dir)
    plt.close('all')
    plt.figure(figsize=(16, 9), dpi=160)
    test_path.plot_cut_path()
    plt.axis('equal')
    plt.savefig(os.path.join(output_dir, 'final_cut_path_jet.png'))
    plt.show()

    tool_path = ToolPath.create_tool_path_from_cut_path(cut_path=test_path, wire_cutter=wire_cutter)
    tool_path.zero_forwards_path_for_cutting()
    gcode = GCodeGenerator(wire_cutter, travel_type=TravelType.CONSTANT_RATIO)
    gcode.create_relative_gcode(file_path=r'./assets/GCode/%s_test.txt' % 'jet', tool_path=tool_path)

    plt.close('all')
    plt.figure(figsize=(16, 9), dpi=400)
    tool_path.plot_tool_paths()
    plt.axis('equal')
    plt.savefig(os.path.join(output_dir, 'final_tool_path_jet.png'))
    plt.show()



main()
