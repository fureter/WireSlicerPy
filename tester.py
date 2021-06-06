import logging
import sys
import copy

import numpy as np
import matplotlib.pyplot as plt

import wire_slicer
from slicer.wire_cutter import WireCutter
from slicer.tool_path import ToolPath
from geometry.parser import Dat
from geometry.primative import Spline
from geometry.primative import Plane
from geometry.primative import GeometricFunctions
from geometry.complex import STL
from geometry.complex import CutPath
from geometry.complex import WingSegment
from geometry.spatial_manipulation import PointManip
from g_code.generator import GCodeGenerator
from g_code.generator import CutLayout
from g_code.generator import TravelType


def main():
    wire_slicer.setup()

    logger = logging.getLogger()

    file_path = r'assets/Airfoils/MH70.dat'
    naca0009_file_path = r'assets/Airfoils/naca0009.dat'

    ag35_file_path = r'assets/Airfoils/ag35.dat'
    s5020_file_path = r'assets/Airfoils/s5020.dat'

    ag35_data = Dat(filepath=ag35_file_path)
    #ag35_data.plot_points_2d()

    ag35_reord = Dat(data=PointManip.reorder_2d_cw(copy.deepcopy(ag35_data.get_data())))
    #ag35_reord.plot_points_2d()

    ag35_path = Spline(ag35_reord.get_data(), resolution=2).get_path()

    logger.info('Profile Name: %s' % ag35_data.name)

    wing = WingSegment(name=ag35_data.name, logger=logger)
    wing.set_span(600)
    wing.set_root_chord(200)
    wing.set_tip_chord(140)
    wing.set_root_airfoil(copy.deepcopy(ag35_path))
    wing.set_tip_airfoil(copy.deepcopy(ag35_path))
    wing.set_washout(1.0)
    wing.set_sweep(1.0)
    wing.symmetric = True

    wire_len = 1000

    wire_cutter = WireCutter(wire_length=wire_len, max_height=300.0, max_speed=100.0, min_speed=0.01,
                             release_height=100.0,
                             start_height=25.0, start_depth=20.0)

    wire_cutter.set_gcode_statup(g_code=['G17', 'G21'])

    wing.prep_for_slicing(plot=True)
    plt.show()
    wing.center_to_wire_cutter(wire_cutter=wire_cutter)
    wing.plot_planform()
    plt.show()
    print('Root Airfoil Point after prepping: %s' % wing.root_airfoil[0])

    cut_path = CutPath.create_cut_path_from_wing(wing, wire_cutter=wire_cutter)
    cut_path.plot_section_link_connections()
    cut_path.plot_cut_path()

    tool_path = ToolPath.create_tool_path_from_cut_path(cut_path=cut_path, wire_cutter=wire_cutter)
    tool_path.zero_forwards_path_for_cutting()
    tool_path.plot_tool_paths()
    tool_path.plot_tool_path_connections(step=3)
    plt.show()

    gcode = GCodeGenerator(wire_cutter, logger, travel_type=TravelType.CONSTANT_RATIO)
    gcode.create_relative_gcode(file_path=r'./assets/GCode/%s_left_test.txt' % wing.name, tool_path=tool_path)

    plt.axis('equal')
    plt.show()

    wing.flip_tip_and_root()
    cut_path = CutPath.create_cut_path_from_wing(wing, wire_cutter)
    tool_path = ToolPath.create_tool_path_from_cut_path(cut_path, wire_cutter=wire_cutter)
    tool_path.zero_forwards_path_for_cutting()
    gcode.create_relative_gcode(file_path=r'./assets/GCode/%s_right_test.txt' % wing.name, tool_path=tool_path)
    plt.axis('equal')
    plt.show()

    # plt.legend(['goe430 @ %smm' % profile_2_dist, 'naca0009 @ %smm' % profile_1_dist, 'tool_path XY @ %smm' % wire_len,
    #             'tool_path UZ @ 0mm'])
    # plt.axis('equal')
    # plt.show()

    # test_stl = STL(file_path=r'./assets/STLs/drop_ship_85sweep.stl',
    #                logger=logger, units='m')
    # test_stl.mesh.convert_units('mm')
    # test_stl.plot_stl()
    # normal = np.array([1, 0, 0])
    # bounding_box = test_stl.mesh.bounds
    # logger.info('Bounds: %s' % bounding_box)
    # scale = 0.999 if bounding_box[0][0] < 0 else 1.001
    # extent = bounding_box[0] * normal*scale
    # slice_plane = Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])
    # test_stl.slice_into_cross_sections(origin_plane=slice_plane, spacing=25)
    # test_stl.plot_cross_sections(bounding_box)
    #
    # plt.show()


main()
