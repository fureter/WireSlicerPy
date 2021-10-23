import logging
import os
import copy

import matplotlib
import matplotlib.pyplot as plt

import slicer.slice_manager as sm
import wire_slicer
from geometry.primative import WorkPiece
from slicer.wire_cutter import WireCutter
from geometry.parser import Dat
from geometry.spatial_manipulation import PointManip
from geometry.complex import WingSegment
from geometry.complex import CutPath
from slicer.tool_path import ToolPath
from g_code.generator import GCodeGenerator
from g_code.generator import TravelType


def main():
    wire_slicer.setup(logger_level=logging.DEBUG)

    matplotlib.use('SVG')

    logger = logging.getLogger(__name__)
    output_dir = r'M:\Projects\CNCHotWireCutter\WireSlicerPy\tests'
    # ==================================================================================================================
    file_path = r'assets/Airfoils/MH60.dat'

    mh60_data = Dat(filepath=file_path)
    # mh60_data.plot_points_2d()

    mh60_reord = Dat(data=PointManip.reorder_2d_cw(copy.deepcopy(mh60_data.get_data()), method=0))
    mh60_reord.plot_points_2d()
    plt.show()
    #ag35_reord.plot_points_2d()

    # naca0009_path = Spline(mh60_reord.get_data(), resolution=4).get_path()

    logger.info('Profile Name: %s' % mh60_data.name)
    #
    wing = WingSegment(name='MicroDelta_MH60', logger=logger)
    wing.set_span(100)
    wing.set_root_chord(110)
    wing.set_tip_chord(70)
    wing.set_root_airfoil(copy.deepcopy(mh60_reord.get_data()))
    wing.set_tip_airfoil(copy.deepcopy(mh60_reord.get_data()))
    wing.set_washout(0)
    wing.set_sweep(6)
    #
    #
    wire_len = 245
    wire_cutter = WireCutter(wire_length=wire_len, max_height=300.0, max_speed=100.0, min_speed=0.01,
                             release_height=100.0,
                             start_height=10.0, start_depth=20.0)
    wire_cutter.set_kerf(kerf=1.2, max_kerf=1.5)

    wire_cutter.set_gcode_statup(g_code=['G17', 'G21'])
    #
    # wing.prep_for_slicing()
    # wing.center_to_wire_cutter(wire_cutter=wire_cutter)
    # cut_path = CutPath.create_cut_path_from_wing(wing, wire_cutter=wire_cutter, debug_kerf=True)
    # tool_path = ToolPath.create_tool_path_from_cut_path(cut_path=cut_path, wire_cutter=wire_cutter)
    # tool_path.zero_forwards_path_for_cutting()
    #
    # gcode = GCodeGenerator(wire_cutter, travel_type=TravelType.CONSTANT_RATIO)
    # gcode.create_relative_gcode(file_path=os.path.join(output_dir, '%s_left.txt' % wing.name), tool_path=tool_path)
    # wing.flip_tip_and_root()
    # cut_path = CutPath.create_cut_path_from_wing(wing, wire_cutter)
    # tool_path = ToolPath.create_tool_path_from_cut_path(cut_path, wire_cutter=wire_cutter)
    # tool_path.zero_forwards_path_for_cutting()
    # gcode.create_relative_gcode(file_path=os.path.join(output_dir, '%s_right.txt' % wing.name), tool_path=tool_path)
    # ==================================================================================================================
    # plt.legend(['goe430 @ %smm' % profile_2_dist, 'naca0009 @ %smm' % profile_1_dist, 'tool_path XY @ %smm' % wire_len,
    #             'tool_path UZ @ 0mm'])
    # plt.axis('equal')
    # plt.show()
    # work_piece = WorkPiece(width=500, height=300, thickness=51)
    # slice_manager = sm.SliceManager(work_piece=work_piece, wire_cutter=wire_cutter)
    #
    # stl_path = os.path.join(os.path.dirname(__name__), r'./assets/STLs/Fuselage_jet.stl')
    # slice_manager.stl_to_gcode(stl_path=stl_path, name='TestJet', output_dir=output_dir, subdivisions=4, units='mm',
    #                            wall_thickness=12)
    # ==================================================================================================================
    work_piece = WorkPiece(width=300, height=300, thickness=14)
    slice_manager = sm.SliceManager(work_piece=work_piece, wire_cutter=wire_cutter)

    stl_path = os.path.join(os.path.dirname(__name__), r'./assets/STLs/FuseMicroDeltaV3.stl')
    slice_manager.stl_to_gcode(stl_path=stl_path, name='MicroDelta_V3', output_dir=output_dir, subdivisions=0, units='mm',
                               wall_thickness=0)
    # ==================================================================================================================

    # r, c = util.util_functions.get_r_and_c_from_num(len(section_list))
    # for ind in range(len(section_list)):
    #     plt.subplot(r, c, ind + 1)
    #     section_list[ind].plot_subdivide_debug(0, 0)
    #     plt.axis('equal')
    # plt.show()


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

    # GeometricFunctions.center_path(issue_sdl.get_path()[0])

main()
