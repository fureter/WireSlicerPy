"""
    Modules to handle differnent functionality revolving around CNC Hotwire cutting and generating slices from 3D
    test_geometry for template creation.

    Goal: Streamline the process of manufacturing complex 3D objects using CNC hotwire cutters, or by generating
    printable templates.

    Functions:
    1. Parse DXF and various ascii formats to read and export spline data to represent cross-sections
    2. Manipulate closed splines:
        -Break closed shapes into sub areas
        -Offset splines to create hollow shapes
        -Transform splines in 3d space
    3. Group closed spline sets to create a cut profile on two parallel offset planes
    4. Optimally Place close spline groups onto a 3d plane to minimize uncovered space
    5. Generate an movement path for a CNC Hotwire cutter that is optimized based on different contraint inputs
    6. Convert spline group placements on a plane to G-Code to move two independent CNC Gantries
    7. Parse STL files representing closed 3D bodies to generate spline groups

    Dependencies:
    -Numpy
    -Scipy
    -matplotlib
    -trimesh
    -shapely
    -jsonpickle
    -GitPython
    -networkx

    Optional:
    -coverage
    -pylint

"""

import logging
import sys

import project_manager as pm

sys.coinit_flags = 0x0
import tkinter as tk

import matplotlib
matplotlib.use("svg")

import gui.window


DEFAULT_AIRFOIL_PATH = r'assets/Airfoils/'
DEFAULT_CAD_PATH = r'assets/STLs/'


def setup(logger_level=logging.INFO):
    logger = logging.getLogger()
    formatter_console = CustomFormatter()
    formatter_file = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)")
    logger.setLevel(logging.DEBUG)

    sys_handler = logging.StreamHandler(sys.stdout)
    sys_handler.setFormatter(formatter_console)
    sys_handler.setLevel(logger_level)
    logger.addHandler(sys_handler)

    file_handler = logging.FileHandler(filename=r'.\DEBUG.txt', mode='wt')
    file_handler.setFormatter(formatter_file)
    file_handler.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)

    logging.getLogger('matplotlib.font_manager').disabled = True
    logging.getLogger('matplotlib.pyplot').disabled = True
    logging.getLogger('matplotlib').disabled = True
    logging.getLogger('shapely.geos').disabled = True
    logging.getLogger('trimesh').disabled = True


class CustomFormatter(logging.Formatter):
    """
    Logging Formatter to add colors and count warning / errors
    from: https://stackoverflow.com/questions/384076/how-can-i-color-python-logging-output
    """
    grey = "\x1b[38;21m"
    bright_blue = "\x1b[94;21m"
    magenta = "\x1b[35;21m\x1b[1m"
    bright_yellow = "\x1b[94;21m"
    red = "\x1b[31;1m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s (File "%(pathname)s", line %(lineno)d)'

    FORMATS = {
        logging.DEBUG: bright_blue + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: bright_yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


def setup_gui(main_window):
    # Temp fake setup to test
    main_window.embedded_windows[gui.window.WindowState.HOME].fill_recent_projects(
        main_window.project_manager.get_recent_projects()
    )


if __name__ == '__main__':
    # setup logger and logger handlers
    setup()
    # Create the main window
    main_window = gui.window.MainWindow('WireSlicerPy', 800, 610,
                                        project_manager=pm.ProjectManager(ini_file=r'./WireSlicerPy.json'))
    # Fake Initilize the recent projects
    setup_gui(main_window)
    # Start the tk main loop
    tk.mainloop()
