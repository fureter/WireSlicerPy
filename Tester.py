import logging
import sys

import numpy as np
import matplotlib.pyplot as plt

from Slicer.WireCutter import WireCutter
from Slicer.ToolPath import ToolPath
from Geometry.Parser import Dat
from Geometry.PrimativeGeometry import Spline
from Geometry.SpatialManipulation import PointManip
from GCode.Generator import GCodeGenerator
from GCode.Generator import TravelType


def main():
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    logger.addHandler(handler)

    logging.getLogger('matplotlib.font_manager').disabled = True

    file_path = r'C:\Users\FurEt\Documents\goe430_refined.dat'
    e66_file_path = r'C:\Users\FurEt\Documents\naca0009.dat'

    goe430 = Dat(filepath=file_path)

    e66 = Dat(filepath=e66_file_path)
    #logger.info('Printing e66 coordinates')
    #e66.log(logger)
    e66_reorder = PointManip.reorder_2d_cw(e66.get_data())
    profile_2_dist = 900
    PointManip.Transform.scale(e66_reorder, np.array([120, 120, 1]), np.array([0.0, 0, 0]))
    PointManip.Transform.rotate(e66_reorder, np.array([0.0, 0, np.deg2rad(2)]))
    PointManip.Transform.translate(e66_reorder, np.array([0, 0, profile_2_dist]))

    e66_spline = Spline(e66_reorder, closed_loop=True)

    # goe614.log(logger)

    # goe614.plot_points2D()
    profile_1_dist = 100
    re_odr_pnts = PointManip.reorder_2d_cw(goe430.get_data())
    PointManip.Transform.scale(re_odr_pnts, np.array([140, 140, 1]), np.array([0.0, 0, 0]))
    PointManip.Transform.translate(re_odr_pnts, np.array([0, 0, profile_1_dist]))
    goe430_spline = Spline(re_odr_pnts, closed_loop=True)

    wire_len = 1000

    wire_cutter = WireCutter(wire_length=wire_len, max_height=300.0, max_speed=25.0, min_speed=1.0, release_height=100.0,
                             start_height=0.0, start_depth=10.0)

    spline_1_goe430 = goe430_spline
    spline_2_e66 = e66_spline

    spline_1_goe430.plot_spline()
    spline_2_e66.plot_spline()

    tool_path = ToolPath.create_tool_path_from_two_splines(spline_1_goe430, spline_2_e66, wire_cutter)
    tool_path.plot_tool_paths()

    gcode = GCodeGenerator(wire_cutter, logger, travel_type=TravelType.CONSTANT_RATIO)
    gcode.create_relative_gcode(file_path=r'M:\Projects\CNCHotWireCutter\test_gcode\test.txt', tool_path=tool_path)

    plt.legend(['goe430 @ %smm' % profile_2_dist, 'naca0009 @ %smm' % profile_1_dist, 'tool_path XY @ %smm' % wire_len, 'tool_path UZ @ 0mm'])
    plt.axis('equal')
    plt.show()




main()
