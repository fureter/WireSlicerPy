import unittest

import numpy as np
import matplotlib.pyplot as plt
from geometry.complex import CrossSectionPair
from geometry.complex import CrossSection
from geometry.complex import CutPath
from geometry.complex import STL
from geometry.complex import WingSegment

import geometry.primative as prim


class TestCrossSectionAndPair(unittest.TestCase):
    def setUp(self):
        self.radius1 = 100
        self.radius2 = 200
        self.num_points = 50
        self.spacing = 10
        self.thickness = 10

        self.points1 = list()
        self.points2 = list()
        # Circle needs to be in clockwise order so that the proper direction is applied when creating the inner hole
        for i in reversed(range(self.num_points)):
            self.points1.append(prim.Point(self.radius1 * np.cos(2*np.pi*i/self.num_points),
                                           self.radius1 * np.sin(2*np.pi*i/self.num_points),
                                           self.radius1 * 0))

            self.points2.append(prim.Point(self.radius2 * np.cos(2*np.pi*i/self.num_points),
                                           self.radius2 * np.sin(2*np.pi*i/self.num_points),
                                           self.radius2 * 0))

        self.section1 = CrossSection(prim.Path(self.points1))
        self.section2 = CrossSection(prim.Path(self.points2))

        self.section3 = CrossSection(prim.Path(self.points1))
        self.section4 = CrossSection(prim.Path(self.points2))

        self.section3.add_simple_hole_from_offset(self.thickness)
        self.section4.add_simple_hole_from_offset(self.thickness)

        self.section_pair = CrossSectionPair(self.section1, self.section2)
        self.section_pair_2 = CrossSectionPair(self.section1, self.section2)

        self.section3.plot(scatter=True)

        plt.axis('equal')
        # plt.show()

    def test_get_path(self):
        path1, path2 = self.section_pair.get_path()
        self.assertListEqual(path1, self.points1)
        self.assertListEqual(path2, self.points2)

    def test_add_hole(self):
        inner_radius_1 = self.radius1 - self.thickness
        inner_radius_2 = self.radius2 - self.thickness

        for point in self.section3.get_path_hole():
            radius = np.sqrt(point**2)
            self.assertTrue(np.abs(radius - inner_radius_1) < 1e-8 or np.abs(radius - self.radius1) < 1e-8)

        for point in self.section4.get_path_hole():
            radius = np.sqrt(point**2)
            self.assertTrue(np.abs(radius - inner_radius_2) < 1e-8 or np.abs(radius - self.radius2) < 1e-8)

    def test_subdivision(self):
        num_sections = 4
        subdivied_list = CrossSectionPair.subdivide_list(num_sections=num_sections,
                                                         section_list=[self.section_pair, self.section_pair_2])
        self.assertTrue(len(subdivied_list) == num_sections * 2)

class TestCutPath(unittest.TestCase):
    pass


class TestSTL(unittest.TestCase):
    pass


class TestWingSegment(unittest.TestCase):
    pass
