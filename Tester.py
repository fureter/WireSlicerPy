import logging
import sys

import numpy as np
import matplotlib.pyplot as plt

from Slicer.WireCutter import WireCutter
from Slicer.ToolPath import ToolPath
from Geometry.Parser import Dat
from Geometry.PrimativeGeometry import Spline
from Geometry.PrimativeGeometry import Plane
from Geometry.ComplexGeometry import STL
from Geometry.ComplexGeometry import WingSegment
from Geometry.SpatialManipulation import PointManip
from GCode.Generator import GCodeGenerator
from GCode.Generator import CutLayout
from GCode.Generator import TravelType


def main():
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    logger.addHandler(handler)

    logging.getLogger('matplotlib.font_manager').disabled = True

    file_path = r'./Assets/Airfoils/MH70.dat'
    naca0009_file_path = r'./Assets/Airfoils/naca0009.dat'

    ag35_file_path = r'./Assets/Airfoils/ag35.dat'

    # ag35_data = Dat(filepath=ag35_file_path)
    # ag35_data.log(logger)
    # ag35_reord = Dat(data=PointManip.reorder_2d_cw(ag35_data.get_data()))
    # ag35_spline = Spline(ag35_reord.get_data())
    # ag35_spline.plot_spline()
    # plt.axis('equal')
    # plt.show()
    #
    # wing = WingSegment(name='AG35_eclipson', logger=logger)
    # wing.set_span(500)
    # wing.set_root_chord(140)
    # wing.set_tip_chord(120)
    # wing.set_root_airfoil(ag35_spline.get_points(resolution=1))
    # wing.set_tip_airfoil(ag35_spline.get_points(resolution=1))
    #
    # wire_len = 1000
    #
    # wire_cutter = WireCutter(wire_length=wire_len, max_height=300.0, max_speed=25.0, min_speed=0.01,
    #                          release_height=100.0,
    #                          start_height=0.0, start_depth=10.0)
    #
    # wire_cutter.set_gcode_statup(g_code=['G17', 'G21'])
    #
    # wing.prep_for_slicing()
    # wing.center_to_wire_cutter(wire_cutter=wire_cutter)
    #
    # tool_path = ToolPath.create_tool_path_from_wing_segment(wing, wire_cutter=wire_cutter)
    # tool_path.plot_tool_path_connections(step=10)
    # tool_path.plot_tool_paths()
    # plt.axis('equal')
    # plt.show()
    #
    # gcode = GCodeGenerator(wire_cutter, logger, travel_type=TravelType.CONSTANT_RATIO)
    # gcode.create_relative_gcode(file_path=r'./Assets/GCode/test.txt', tool_path=tool_path,
    #                             key_points=tool_path.get_key_points_for_wing(),
    #                             cut_mode=CutLayout.SPLIT_SEGMENT)
    # plt.axis('equal')
    # plt.show()
    #
    # plt.legend(['goe430 @ %smm' % profile_2_dist, 'naca0009 @ %smm' % profile_1_dist, 'tool_path XY @ %smm' % wire_len,
    #             'tool_path UZ @ 0mm'])
    # plt.axis('equal')
    # plt.show()

    test_stl = STL(file_path=r'./Assets/STLs/hybridPlane.stl',
                   logger=logger, units='m')
    test_stl.mesh.convert_units('mm')
    test_stl.plot_stl()
    normal = np.array([1, 0, 0])
    bounding_box = test_stl.mesh.bounds
    logger.info('Bounds: %s' % bounding_box)
    scale = 0.999 if bounding_box[0][0] < 0 else 1.001
    extent = bounding_box[0] * normal*scale
    slice_plane = Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])
    test_stl.slice_into_cross_sections(origin_plane=slice_plane, spacing=25)
    test_stl.plot_cross_sections()

    plt.show()


main()
