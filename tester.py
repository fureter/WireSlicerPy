import copy
import logging
import os

import matplotlib.pyplot as plt

# import gui.window
import numpy as np
import numpy.fft

import wire_slicer
from g_code.generator import GCodeGenerator, TravelType
from geometry.complex import WingSegment, CutPath
import geometry.primative as prim
from geometry.parser import Dat
import slicer.slice_manager as sm
from geometry.spatial_manipulation import PointManip
from slicer.tool_path import ToolPath
from slicer.wire_cutter import WireCutter


def main():
    wire_slicer.setup(logger_level=logging.DEBUG)
    # main_window = gui.window.MainWindow('Test Gui', width=800, height=600)
    # tkinter.mainloop()
    #
    # matplotlib.use('SVG')

    # test_json = os.path.join(r'M:\Projects\CNCHotWireCutter\WireSlicerPy\tests', 'Nebula_V2_Fuse', 'json',
    #                          'spatial_placement_bf_cp_gen.json')
    #
    # sp = serializer.decode(test_json)
    # indexes = list()
    # for val in sp.state_dict:
    #     indexes.append(val)
    # for item in indexes:
    #     sp.state_dict[int(item)] = sp.state_dict[item]
    #     del sp.state_dict[item]
    # for cut in range(1, sp.num_sections+1):
    #     prim.GeometricFunctions.plot_cross_sections_on_workpiece(sp.state_dict, sp.work_piece, os.path.join(r'M:\Projects\CNCHotWireCutter\WireSlicerPy\tests', 'Nebula_V2_Fuse','plots'),
    #                                                              index='final', num_sections=cut)
    # cut_path_1, cut_path_2 = sp.create_section_links_for_cross_section_pairs(method=1)
    # sp.plot_section_order(os.path.join(r'M:\Projects\CNCHotWireCutter\WireSlicerPy\tests', 'Nebula_V2_Fuse','plots'))
    logger = logging.getLogger(__name__)
    # cut_paths = list()
    # for ind in range(sp.num_sections):
    #     cut_paths.append(CutPath(cut_path_1[ind], cut_path_2[ind]))
    # for ind, cut_path in enumerate(cut_paths):
    #     plt.close('all')
    #     plt.figure(figsize=(16, 9), dpi=360)
    #     cut_path.plot_cut_path()
    #     plt.axis('equal')
    #     plt.savefig(os.path.join(os.path.join(r'M:\Projects\CNCHotWireCutter\WireSlicerPy\tests', 'Nebula_V2_Fuse','plots'), '%s_final_cut_path_%s.png' % ('Nebula_V2_Fuse', ind)))
    #     plt.show()
    #     logger.info('Cut Paths Valid: %s', cut_path.is_valid_cut_path())

    output_dir = r'M:\Projects\CNCHotWireCutter\WireSlicerPy\tests'
    # ==================================================================================================================
    file_path = r'assets/Airfoils/MH60.dat'

    mh60_data = Dat(filepath=file_path)
    # mh60_data.plot_points_2d()

    mh60_reord = Dat(data=PointManip.reorder_2d_cw(copy.deepcopy(mh60_data.get_data()), method=7))
    mh60_reord.plot_points_2d()
    kerf = prim.Path(prim.GeometricFunctions.parallel_curve(mh60_reord.get_data(), 0.01, 1))
    prim.GeometricFunctions.plot_path(kerf.get_path(), 'c1')
    plt.axis('equal')
    plt.show()
    # ag35_reord.plot_points_2d()

    # naca0009_path = Spline(mh60_reord.get_data(), resolution=4).get_path()

    # logger.info('Profile Name: %s' % mh60_data.name)
    #
    # wing = WingSegment(name='dan_hole_test_feed_comp', logger=logger)
    # wing.set_span(420)
    # wing.set_root_chord(180)
    # wing.set_tip_chord(180)
    # wing.set_root_airfoil(copy.deepcopy(prim.GeometricFunctions.normalize_path_points(mh60_reord.get_data(),
    #                                                                                   num_points=128)))
    # wing.set_tip_airfoil(copy.deepcopy(prim.GeometricFunctions.normalize_path_points(mh60_reord.get_data(),
    #                                                                                  num_points=128)))
    # wing.set_washout(0)
    # wing.set_sweep(0)
    #
    # spar_width = 2.3
    # wing.add_hole_root(WingSegment.RectangularHole(chord_start=0.25, height=9.6/2, width=spar_width, offset=0, angle=0, start_angle=-90))
    # wing.add_hole_root(WingSegment.RectangularHole(chord_start=0.25, height=9.6/2, width=spar_width, offset=0, angle=0, start_angle=90))
    # wing.add_hole_tip(WingSegment.RectangularHole(chord_start=0.25, height=9.6/2, width=spar_width, offset=0, angle=0,
    #                                               start_angle=-90))
    # wing.add_hole_tip(
    #     WingSegment.RectangularHole(chord_start=0.25, height=9.6/2, width=spar_width, offset=0, angle=0, start_angle=90))
    # # #
    # wing.add_hole_root(WingSegment.RectangularHole(chord_start=0.6, height=9.6/2, width=spar_width, offset=0, angle=0, start_angle=-90))
    # wing.add_hole_root(WingSegment.RectangularHole(chord_start=0.6, height=9.6/2, width=spar_width, offset=0, angle=0, start_angle=90))
    # wing.add_hole_tip(WingSegment.RectangularHole(chord_start=0.6, height=9.6/2, width=spar_width, offset=0, angle=0,
    #                                               start_angle=-90))
    # wing.add_hole_tip(
    #     WingSegment.RectangularHole(chord_start=0.6, height=9.6/2, width=spar_width, offset=0, angle=0, start_angle=90))
    # wing.add_hole_root(WingSegment.CircleHole(chord_start=0.2, radius=2.5, offset=5, angle=0, start_angle=90))
    # wing.add_hole_tip(WingSegment.CircleHole(chord_start=0.2, radius=2.5, offset=3, angle=0, start_angle=90))

    # wing.add_hole_root(
    #     WingSegment.CavityHole(chord_start=0.35, chord_stop=0.5, airfoil_path=copy.deepcopy(mh60_reord.get_data()),
    #                            thickness=6.5,
    #                            num_points=254))
    # #
    # wing.add_hole_tip(
    #     WingSegment.CavityHole(chord_start=0.35, chord_stop=0.5, airfoil_path=copy.deepcopy(mh60_reord.get_data()),
    #                            thickness=6.5,
    #                            num_points=254))
    #
    # serializer.encode(wing, output_dir, 'wing')
    # del wing
    # wing = serializer.decode(os.path.join(output_dir, 'wing.json'))

    wire_len = 245
    wire_cutter = WireCutter(wire_length=wire_len, max_height=300.0, max_speed=200.0, min_speed=120,
                             release_height=100.0,
                             start_height=6.0, start_depth=20.0, name='base', dynamic_tension=True, max_depth=300)
    wire_cutter.set_kerf(kerf=1.0, max_kerf=1.2)
    wire_cutter.set_dynamic_tension_motor_letter('V')
    wire_cutter.reverse_dynamic_tension(True)
    wire_cutter.set_dynamic_tension_feed_comp(True)

    wire_cutter.set_gcode_statup(g_code=['G17', 'G21'])

    # sm.SliceManager.wing_to_gcode(wing, wire_cutter, output_dir)
    #
    # wing.prep_for_slicing(plot=True)
    # # plt.show()
    # wing.center_to_wire_cutter(wire_cutter=wire_cutter)
    # cut_path = CutPath.create_cut_path_from_wing(wing, wire_cutter=wire_cutter, debug_kerf=True, output_dir=output_dir)
    # tool_path = ToolPath.create_tool_path_from_cut_path(cut_path=cut_path, wire_cutter=wire_cutter)
    # tool_path.zero_forwards_path_for_cutting()
    # # # #
    # gcode = GCodeGenerator(wire_cutter, travel_type=TravelType.CONSTANT_RATIO)
    # gcode.create_relative_gcode(file_path=os.path.join(output_dir, '%s_hole_test.txt' % wing.name), tool_path=tool_path, wire_cutter=wire_cutter)
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
    work_piece = prim.WorkPiece(width=330, height=200, thickness=36.5)
    # slice_manager = sm.SliceManager(work_piece=work_piece, wire_cutter=wire_cutter)
    #
    stl_path = r"M:\Projects\50mm_edf_jet\Fuse_rear.stl"
    # hollow_section_list = [2, 3, 4, 5, 6]
    sm.SliceManager.stl_to_gcode(stl_path=stl_path, name='50mm_edf_jet', output_dir=output_dir, subdivisions=3,
                                 units='mm',
                                 wall_thickness=10, section_gap=3, open_nose=True, work_piece=work_piece,
                                 wire_cutter=wire_cutter)  # hollow_section_list=hollow_section_list)


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
