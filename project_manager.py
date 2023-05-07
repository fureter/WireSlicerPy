import os
import json

import geometry.parser as gp
import geometry.primative as prim
import geometry.spatial_manipulation as sm
import wire_slicer
import serializer


class ProjectManager(object):
    def __init__(self, ini_file):
        self.ini_file = ini_file
        self.recent_projects = dict()
        self.program_airfoil_dirs = [wire_slicer.DEFAULT_AIRFOIL_PATH]
        self.program_cad_dirs = [wire_slicer.DEFAULT_CAD_PATH]
        self.ini_data = None

        self.cad_files = dict()
        self.airfoils = dict()

        self.load_project_manager()

    def load_project_manager(self):
        if os.path.exists(self.ini_file):
            with open(self.ini_file, 'rt') as json_file:
                json_text = json_file.read()
            data = json.JSONDecoder().decode(s=json_text)
            self.recent_projects = data.get('projects', dict())
            self.program_airfoil_dirs = data.get('program_airfoils', list())
            self.program_cad_dirs = data.get('program_cad', list())
        self._load_from_directories()

    def save_ini_file(self):
        with open(self.ini_file, 'wt') as json_file:
            json_file.write(json.JSONEncoder().encode({'projects': self.recent_projects,
                                                       'program_airfoils': self.program_airfoil_dirs,
                                                       'program_cad': self.program_cad_dirs}))

    def get_recent_projects(self):
        recent_project_list = list()
        if self.recent_projects is not None:
            for key, item in self.recent_projects.items():
                recent_project_list.append((key, item))

        return recent_project_list

    def default_project(self):
        return Project(r'./', 'Default')

    def update_project_list(self, project):
        """
        :param Project project:
        """
        if project.name not in self.recent_projects.keys():
            self.recent_projects[project.name] = project.file_path

    def _load_from_directories(self):
        for directory in self.program_airfoil_dirs:
            if os.path.exists(directory):
                for file in os.listdir(directory):
                    if file.endswith(".dat"):
                        name = file[:-4]
                        self.airfoils[name] = sm.PointManip.reorder_2d_cw(
                            gp.Dat(filepath=os.path.join(directory, file)).get_data(), method=7)

        for directory in self.program_cad_dirs:
            if os.path.exists(directory):
                for file in os.listdir(directory):
                    if file.endswith(".stl"):
                        name = file[:-4]
                        self.cad_files[name] = os.path.join(directory, file)
                    elif file.endswith(".obj"):
                        name = file[:-4]
                        self.cad_files[name] = os.path.join(directory, file)

    def add_new_cad_directory(self, directory):
        self.program_cad_dirs.append(directory)
        for file in os.listdir(directory):
            if file.endswith(".stl"):
                name = file[:-4]
                self.cad_files[name] = os.path.join(directory, file)
            elif file.endswith(".obj"):
                name = file[:-4]
                self.cad_files[name] = os.path.join(directory, file)

    def add_new_airfoil_directory(self, directory):
        self.program_airfoil_dirs.append(directory)
        for file in os.listdir(directory):
            if file.endswith(".dat"):
                name = file[:-4]
                self.airfoils[name] = sm.PointManip.reorder_2d_cw(
                    gp.Dat(filepath=os.path.join(directory, file)).get_data(), method=7)


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

    @property
    def file_path(self):
        return os.path.join(self.output_dir, '%s.proj' % self.name)


class PartDatabase(object):
    def __init__(self):
        self.airfoil_directory_paths = list()
        self.cad_directory_paths = list()
        self.individual_file_paths = list()
        self.airfoils = dict()
        self.cad_files = dict()

        self._load_from_directories()

    @property
    def directory_paths(self):
        return self.airfoil_directory_paths + self.cad_directory_paths

    def add_new_airfoil_directory(self, directory):
        self.directory_paths.append(directory)
        self.airfoil_directory_paths.append(directory)
        self._load_from_new_directory()

    def add_new_cad_directory(self, directory):
        self.directory_paths.append(directory)
        self.cad_directory_paths.append(directory)
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
                        self.cad_files[name] = os.path.join(directory, file)
                    elif file.endswith(".obj"):
                        name = file[:-4]
                        self.cad_files[name] = os.path.join(directory, file)

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

    def reset(self):
        self.airfoils = dict()
        self.cad_files = dict()

        self._load_from_directories()
