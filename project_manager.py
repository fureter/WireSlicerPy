import os

import geometry.parser as gp
import geometry.primative as prim
import geometry.spatial_manipulation as sm
import wire_slicer
import serializer


class Project(object):
    def __init__(self, output_dir, name):
        self.output_dir = output_dir
        self.database = PartDatabase()
        self.name = name

        self.machines = list()
        self.wings = list()
        self.cad_parts = list()

    def save_project(self):
        serializer.encode(self, self.output_dir, '%s.proj' % self.name)

    @staticmethod
    def load_project(file_path):
        return serializer.decode(file_path)

    @staticmethod
    def default_project():
        return Project(r'./', 'Default')


class PartDatabase(object):
    def __init__(self):
        self.directory_paths = [wire_slicer.DEFAULT_AIRFOIL_PATH, wire_slicer.DEFAULT_CAD_PATH]
        self.individual_file_paths = list()
        self.airfoils = dict()
        self.cad_files = dict()

        self._load_from_directories()

    def add_new_directory(self, directory):
        self.directory_paths.append(directory)
        self._load_from_new_directory()

    def add_file(self, file_path):
        self.individual_file_paths.append(file_path)
        self._load_from_file()

    def _load_from_directories(self):
        for directory in self.directory_paths:
            if os.path.exists(directory):
                for file in os.listdir(directory):
                    if file.endswith(".dat"):
                        name = file[:-4]
                        self.airfoils[name] = sm.PointManip.reorder_2d_cw(
                            gp.Dat(filepath=os.path.join(directory, file)).get_data(), method=7)
                    elif file.endswith(".stl"):
                        name = file[:-4]
                        self.cad_files[name] = os.path.join(self.directory_paths[-1], file)
                    elif file.endswith(".obj"):
                        name = file[:-4]
                        self.cad_files[name] = os.path.join(self.directory_paths[-1], file)

    def _load_from_new_directory(self):
        for file in os.listdir(self.directory_paths[-1]):
            if file.endswith(".dat"):
                name = file[:-4]
                self.airfoils[name] = sm.PointManip.reorder_2d_cw(
                    gp.Dat(filepath=os.path.join(self.directory_paths[-1], file)).get_data(), method=7)
            elif file.endswith(".stl"):
                name = file[:-4]
                self.cad_files[name] = os.path.join(self.directory_paths[-1], file)
            elif file.endswith(".obj"):
                name = file[:-4]
                self.cad_files[name] = os.path.join(self.directory_paths[-1], file)

    def _load_from_file(self):
        file_path = self.individual_file_paths[-1]
        file_name = os.path.basename(file_path)
        name = file_name[:-4]
        if file_path.endswith(".dat"):
            self.airfoils[name] = sm.PointManip.reorder_2d_cw(
                gp.Dat(filepath=file_path).get_data(), method=7)
        elif file_path.endswith(".stl"):
            self.cad_files[name] = file_path
        elif file_path.endswith(".obj"):
            self.cad_files[name] = file_path
