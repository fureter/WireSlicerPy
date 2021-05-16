import logging

import numpy as np
import scipy as sp
import trimesh as tm
from matplotlib import pyplot as plt

from .PrimativeGeometry import Point
from .PrimativeGeometry import Plane
from Util import UtilFunctions


class STL(object):
    """

    """

    def __init__(self, file_path, logger=None):
        """

        :param file_path:
        """
        if logger is None:
            logger = logging.getLogger()
        self.logger = logger
        self.cross_sections = None
        self._file_path = file_path
        self._setup()

    def slice_into_cross_sections(self, origin_plane, spacing):
        """

        :param Plane origin_plane:
        :param float spacing:
        :return:
        """
        self.cross_sections = list()
        origin = np.array(origin_plane.origin)
        normal = np.array(origin_plane.normal)
        section = self.mesh.section(plane_origin=origin, plane_normal=normal)
        if section is not None:
            self.cross_sections.append(section)
            next_origin = origin + normal * spacing
            while section is not None:
                section = self.mesh.section(plane_origin=next_origin, plane_normal=normal)
                next_origin = next_origin + normal * spacing
                if section is not None:
                    self.cross_sections.append(section)

            next_origin = origin - normal * spacing
            section = self.mesh.section(plane_origin=next_origin, plane_normal=normal)
            if section is not None:
                self.cross_sections.append(section)
                next_origin = origin + normal * spacing
                while section is not None:
                    section = self.mesh.section(plane_origin=next_origin, plane_normal=normal)
                    next_origin = next_origin - normal * spacing
                    if section is not None:
                        self.cross_sections.append(section)
        else:
            raise AttributeError('Error: Plane with origin(%s) does not intersect the STL' % origin_plane.origin)

    def plot_stl(self):
        self.mesh.show()

    def plot_cross_sections(self):
        if self.cross_sections is None:
            raise AttributeError('Error: Cross sections have not been generated for this STL')
        num_sections = len(self.cross_sections)
        self.logger.info('Number of sections being plotted: %s' % num_sections)
        (r, c) = UtilFunctions.get_r_and_c_from_num(num_sections)
        self.logger.info('Creating section subplots with r: %s and c: %s' % (r, c))
        i = 1
        for section in self.cross_sections:
            ax = plt.subplot(int(r), int(c), i)
            section.to_planar()[0].plot_entities()
            ax.set_title('Cross Section: %s' % i)
            i += 1
            plt.show(block=False)

    def _setup(self):
        tm.util.attach_to_log(level=logging.INFO)
        self.mesh = tm.load(self._file_path, force='mesh')

class Wing(object):

    def __init__(self):