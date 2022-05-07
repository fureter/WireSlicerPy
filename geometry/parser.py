import os

import numpy as np
import matplotlib.pyplot as plt

from .primative import Point


class Parser():
    """Base Parser class to be extended by implemented parser classes for specific file types.

    :param self._data: Array of Point entries representing points along the curve defined by the dat file.
    :type self._data: np.ndarray.
    :param self._filepath: path to .dat file either for import or export.
    :type self._filepath: str.
    """

    def __init__(self, data=None, filepath=None):
        self.name = None
        if data is not None:
            self._data = data
        if filepath is not None:
            self._filepath = filepath
            self._import_data()

    def get_data(self):
        """Return parsed data."""
        return self._data

    def _import_data(self):
        """Abstract function for importing data, implemented by inherited classes."""
        raise NotImplementedError()

    def export_data(self, filepath):
        """Abstract function for exporting data to a file, implemented by inherited classes."""
        raise NotImplementedError()

    def log(self, logger):
        """Log the data from the parser using a logger instance"""
        logger.info('Printing Data for section %s\r\n' % self.name)
        for data in self._data:
            logger.info(str(data))


class Dat(Parser):
    """Parser implementation for .dat files. Can import 2D and 3D dat files, irrespective of point order.
    extension of :class: `Parser`

    :param self._data: Array of Point entries representing points along the curve defined by the dat file.
    :type self._data: np.ndarray.
    :param self._filepath: path to .dat file either for import or export.
    :type self._filepath: str.
    :param self._type: Defines the whether the data is from a 2D or a 3D source.
    :type self._type: str.
    """

    def __init__(self, data=None, filepath=None):
        """Constructor for Dat parser. Accepts either a list of data, or a filepath to import data from.

        :param data: Array of Point entries representing points along the curve defined by the dat file.
        :type data: np.ndarray.
        :param filepath: path to .dat file either for import or export.
        :type filepath: str.
        """
        super().__init__(data, filepath)
        if filepath is None:
            self.type = None
            self.name = None

    def _import_data(self):
        """Implementation of Parser _import_data function. Reads through each line of the .dat file and creates 3D
        points from the data.
        """
        if self._filepath is None:
            raise AttributeError('Filepath is missing for data import')

        points = list()
        self.type = '2D'
        with open(self._filepath) as file:
            for row in file:
                split = row.split(' ')  # most dat files use spaces to separate the values instead of commas.
                coords = list()

                if len(split) == 1:  # if there were no spaces, the file is most likely using \t characters
                    split = split[0].split('\t')  # re-split with \t

                for indx in range(0, len(split)):
                    # check if the value is cast-able as a float (some dat files have headers we want to ignore)
                    if is_float(split[indx]):
                        coords.append(float(split[indx]))
                    # check if the split has a new line character, if so remove it
                    if r'\n' in split[indx]:
                        coords.append(float(split[indx][:-2]))
                # initialize the point coordinates with 0,0,0 this handles the 2D case where z is left as 0
                (x, y, z) = 0, 0, 0
                if len(coords) > 1:
                    x = coords[0]
                    y = coords[1]
                    if x == y and x > 5:
                        continue
                    # check if there are more than 2 coordinates
                    if len(coords) > 2:
                        z = coords[2]
                        self.type = '3D'
                    # add the point to the list
                    points.append(Point(x, y, z))
                else:
                    print('Profile Name: %s' % split[0].rstrip())
                    self.name = split[0].rstrip()
        if self.name is None:
            self.name = os.path.basename(self._filepath).rstrip()
        # convert the list of points to a numpy array and save it to the  as self._data
        self._data = np.array(points)

    def export_data(self, filepath=None):
        """Export data implementation for Dat files.

        :param filepath: Filepath to the file to save the new dat file as. If not provided will use self._filepath.
        """
        # currently not implemented, call super function to raise error
        super().export_data(filepath)

    def plot_points_2d(self):
        """plot the x,y coordinates of the dat file to visualize the file. Assumes the x and y dimensions are the ones
        containing the relevant data, z is assumed empty (not always the case). Function does not show plot, relies on
        higher level implementation to display the plot so that multiple plots can be stacked up.
        """
        size = len(self._data)
        x = np.zeros(size)
        y = np.zeros(size)

        for i in range(0, size):
            x[i] = self._data[i]['x']
            y[i] = self._data[i]['y']
        plt.plot(x, y, 'vr')
        plt.plot(x, y, 'k')

    def plot_points_2d_gui(self, plot1, font_color, tet_color):
        """plot the x,y coordinates of the dat file to visualize the file. Assumes the x and y dimensions are the ones
        containing the relevant data, z is assumed empty (not always the case). Function requires a plot object input
        from matplotlib, this is used with the gui to write to a tkinter canvas.
        """
        size = len(self._data)
        x = np.zeros(size)
        y = np.zeros(size)

        for i in range(0, size):
            x[i] = self._data[i]['x']
            y[i] = self._data[i]['y']
        plot1.plot(x, y, font_color)
        plot1.plot(x, y, 'v', markersize=3, color=tet_color)
        plot1.axis('equal')


def is_float(val):
    """Check whether val is cast-able as a float.

    :param val: Value to check if cast-able as a float.
    :type val: unknown
    :return: True or False as to whether val can be cas-table as a float.
    """
    ret_val = False
    try:
        float(val)
        ret_val = True
    finally:
        return ret_val
