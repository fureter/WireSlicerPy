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
    stl_path = r"assets\STLs\Nose_plug.stl"
    voxel = comp.Voxel.stl_to_voxel(stl_path, units='mm', resolution=1)


if __name__ == '__main__':
    main()
