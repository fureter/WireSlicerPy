import os

import geometry.parser as gp
import geometry.primative as prim
import geometry.spatial_manipulation as sm


class Project(object):
    def __init__(self, airfoil_database, output_dir):
        self.output_dir = output_dir
        self.airfoil_database = airfoil_database


class AirfoilDatabase(object):
    def __init__(self):
        self.directory_paths = [r'assets/Airfoils/']
        self.airfoils = dict()

        self._load_from_directories()

    def _load_from_directories(self):
        for directory in self.directory_paths:
            for file in os.listdir(directory):
                if file.endswith(".dat"):
                    name = file[:-4]
                    self.airfoils[name] = sm.PointManip.reorder_2d_cw(
                        gp.Dat(filepath=os.path.join(directory, file)).get_data(), method=7)
