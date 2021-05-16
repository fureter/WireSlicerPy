import logging
import sys

import numpy as np
import matplotlib.pyplot as plt

import Geometry.PrimativeGeometry
from Slicer.WireCutter import WireCutter
from Slicer.ToolPath import ToolPath
from Geometry.Parser import Dat
from Geometry.PrimativeGeometry import Spline
from Geometry.PrimativeGeometry import Plane
from Geometry.ComplexGeometry import STL
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

    naca0009 = Dat(filepath=e66_file_path)
    # logger.info('Printing e66 coordinates')
    # e66.log(logger)
    naca009_reorder = PointManip.reorder_2d_cw(naca0009.get_data())
    profile_2_dist = 900
    PointManip.Transform.scale(naca009_reorder, np.array([100, 100, 1]), np.array([0.0, 0, 0]))
    PointManip.Transform.rotate(naca009_reorder, np.array([0.0, 0, np.deg2rad(0)]))
    PointManip.Transform.translate(naca009_reorder, np.array([8, 3, profile_2_dist]))

    naca0009_spline = Spline(naca009_reorder, closed_loop=True)

    # goe614.log(logger)

    # goe614.plot_points2D()
    profile_1_dist = 100
    re_odr_pnts = PointManip.reorder_2d_cw(goe430.get_data())
    PointManip.Transform.scale(re_odr_pnts, np.array([140, 140, 1]), np.array([0.0, 0, 0]))
    PointManip.Transform.translate(re_odr_pnts, np.array([0, 0, profile_1_dist]))
    goe430_spline = Spline(re_odr_pnts, closed_loop=True)

    wire_len = 1000

    wire_cutter = WireCutter(wire_length=wire_len, max_height=300.0, max_speed=25.0, min_speed=1.0,
                             release_height=100.0,
                             start_height=0.0, start_depth=10.0)

    spline_1_goe430 = goe430_spline
    spline_2_e66 = naca0009_spline

    spline_1_goe430.plot_spline()
    spline_2_e66.plot_spline()

    tool_path = ToolPath.create_tool_path_from_two_splines(spline_1_goe430, spline_2_e66, wire_cutter)
    tool_path.plot_tool_paths()

    key_points = list()
    key_points.append((Geometry.PrimativeGeometry.GeometricFunctions.get_point_from_max_coord(tool_path._path1, 'x'),
                       Geometry.PrimativeGeometry.GeometricFunctions.get_point_from_max_coord(tool_path._path2, 'x')))

    gcode = GCodeGenerator(wire_cutter, logger, travel_type=TravelType.CONSTANT_RATIO)
    gcode.create_relative_gcode(file_path=r'M:\Projects\CNCHotWireCutter\test_gcode\test.txt', tool_path=tool_path,
                                key_points=key_points)

    plt.legend(['goe430 @ %smm' % profile_2_dist, 'naca0009 @ %smm' % profile_1_dist, 'tool_path XY @ %smm' % wire_len,
                'tool_path UZ @ 0mm'])
    plt.axis('equal')
    plt.show()

    test_stl = STL(file_path=r'M:\Projects\DropShip\OpenFOAM\Sweep_45_0AOA\constant\triSurface\assembly.stl',
                   logger=logger)
    #test_stl.plot_stl()
    normal = np.array([0, 1, 0])
    bounding_box = test_stl.mesh.bounds
    logger.info('Bounds: %s' % bounding_box)
    extent = bounding_box[0] * normal*0.9999
    slice_plane = Plane(extent[0], extent[1], extent[2], normal[0], normal[1], normal[2])
    test_stl.slice_into_cross_sections(origin_plane=slice_plane, spacing=0.01)
    test_stl.plot_cross_sections()

    plt.show()


main()
