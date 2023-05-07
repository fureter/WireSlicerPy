import copy
import logging
import os

import matplotlib.pyplot as plt

# import gui.window
import numpy as np
import numpy.fft

import serializer
import wire_slicer
import geometry.complex as comp
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

    sp = serializer.decode(file_path=r"M:\Projects\40mm_edf_jet\Rear\json\spatial_placement_bf_cp_gen.json")
    cut_path_1, cut_path_2 = sp.create_section_links_for_cross_section_pairs(method=1)
    # work_piece = prim.WorkPiece(400, 200, 14.5)
    # section_gap = 4.0
    # wire_cutter = WireCutter(name='short', wire_length=254, max_height=300, max_depth=600, max_speed=120, min_speed=60,
    #                          feed_rate_mode=94, axis_def='X{:.6f} Y{:.6f} U{:.6f} Z{:.6f}',
    #                          dynamic_tension=False)
    #
    # output_dir = r'M:\Projects\40mm_edf_jet\Rear'

    # cut_paths = comp.CutPath.create_cut_path_from_cross_section_pair_list(subdivision_list, work_piece,
    #                                                                       section_gap=section_gap,
    #                                                                       wire_cutter=wire_cutter,
    #                                                                       output_dir=os.path.join(output_dir,
    #                                                                                               'plots'))

main()
