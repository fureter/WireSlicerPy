import logging
import sys

import numpy as np
import matplotlib.pyplot as plt

from Slicer.WireCutter import WireCutter
from Slicer.ToolPath import ToolPath
from Geometry.Parser import Dat
from Geometry.PrimativeGeometry import Spline
from Geometry.SpatialManipulation import PointManip


def main():
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    logger.addHandler(handler)

    logging.getLogger('matplotlib.font_manager').disabled = True

    file_path = r'C:\Users\FurEt\Documents\goe430_refined.dat.dat'
    e66_file_path = r'C:\Users\FurEt\Documents\e66.dat'

    goe430 = Dat(filepath=file_path)

    e66 = Dat(filepath=e66_file_path)
    logger.info('Printing e66 coordinates')
    e66.log(logger)
    e66_reorder = PointManip.reorder_2d_cw(e66.get_data())

    PointManip.Transform.scale(e66_reorder, np.array([100, 100, 1]), np.array([0.0, 0, 0]))
    PointManip.Transform.translate(e66_reorder, np.array([0, 0, 750]))

    e66_spline = Spline(e66_reorder, closed_loop=True)

    # goe614.log(logger)

    # goe614.plot_points2D()

    re_odr_pnts = PointManip.reorder_2d_cw(goe430.get_data())
    print('before translate: %s' % re_odr_pnts)
    PointManip.Transform.scale(re_odr_pnts, np.array([140, 140, 1]), np.array([0.0, 0, 0]))
    PointManip.Transform.translate(re_odr_pnts, np.array([0, 0, 250]))
    print('after translate: %s ' % re_odr_pnts)
    goe430_spline = Spline(re_odr_pnts, closed_loop=True)

    wire_cutter = WireCutter(wire_length=2000.0, max_height=300.0, max_speed=25.0, min_speed=1.0, release_height=100.0,
                             start_height=50.0)

    spline_1_goe430 = goe430_spline
    spline_2_e66 = e66_spline

    spline_1_goe430.plot_spline()
    spline_2_e66.plot_spline()

    tool_path = ToolPath.create_tool_path_from_two_splines(spline_1_goe430, spline_2_e66, wire_cutter)
    tool_path.plot_tool_paths()

    plt.legend(['goe430 @ 250mm', 'e66 @ 750mm', 'tool_path @ 0mm', 'tool_path @ 2000mm'])
    plt.axis('equal')
    plt.show()


main()
