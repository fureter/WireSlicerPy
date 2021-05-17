import timeit

import GCode.CommandLibrary as CommandLibrary
from Geometry.PrimativeGeometry import Spline
from Slicer.WireCutter import WireCutter
import Slicer.ToolPath as ToolPath


class TravelType(object):
    # Maintains both gantry's at a constant speed.
    CONSTANT_SPEED = 0
    # Maintains a constant speed/total_distance between both gantries
    CONSTANT_RATIO = 1
    # Maintains a constant speed/total_distance between both gantries per toolpath
    ADAPTIVE_RATIO = 2

    RANGE = [CONSTANT_SPEED, CONSTANT_RATIO, ADAPTIVE_RATIO]

    @staticmethod
    def to_str(val):
        return {TravelType.CONSTANT_SPEED: 'CONST_SPEED',
                TravelType.CONSTANT_RATIO: 'CONST_RATIO',
                TravelType.ADAPTIVE_RATIO: 'ADAPT_RATIO'}[val]


class GCodeGenerator(object):
    def __init__(self, wire_cutter, logger, travel_type=TravelType.CONSTANT_SPEED):
        """

        :param WireCutter wire_cutter:
        :param travel_type:
        """
        self._wire_cutter = wire_cutter
        self._travel_type = travel_type
        self.logger = logger

    def set_travel(self, travel_type):
        if travel_type in TravelType.RANGE:
            self._travel_type = travel_type
        else:
            raise NotImplementedError('Selected travel type of %s is not implemented' % travel_type)

    def create_relative_gcode(self, file_path, tool_path, key_points=None):
        """

        :param file_path:
        :param ToolPath tool_path:
        :return:
        """
        self.logger.info('Creating relative gcode with travel type: %s at location: %s' %
                         (TravelType.to_str(self._travel_type), file_path))
        cmd_list = list()

        if self._wire_cutter.start_up_gcode:
            cmd_list.extend(self._wire_cutter.start_up_gcode)

        enum = CommandLibrary.GCodeCommands.PositionMode
        cmd_list.append(enum.set_positioning_mode(enum.RELATIVE))

        enum = CommandLibrary.GCodeCommands.MovementCommand
        start_height = self._wire_cutter.start_height
        start_depth = self._wire_cutter.start_depth
        cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(0.0, start_height, 0.0, start_height)))
        cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_depth, 0.0, start_depth, 0.0)))

        if self._travel_type == TravelType.CONSTANT_SPEED:
            movement_list, start_point1, start_point2 = self.create_constant_speed_movements(tool_path)
        elif self._travel_type == TravelType.CONSTANT_RATIO:
            movement_list, start_point1, start_point2 = self.create_constant_ratio_movements(tool_path, key_points)
        elif self._travel_type == TravelType.ADAPTIVE_RATIO:
            movement_list, start_point1, start_point2 = self.create_adaptive_ratio_movements(None)
        else:
            raise ValueError('Travel Type of %s is invalid' % self._travel_type)

        cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_point1['x'], start_point1['y'],
                                                                              start_point2['x'], start_point2['y'])))

        for movement in movement_list:
            cmd_list.append(enum.g1_linear_move(
                self._wire_cutter.axis_def.format(movement[0], movement[1], movement[2], movement[3])))

        cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(-start_depth, 0.0, -start_depth, 0.0)))

        self._save_gcode_file(file_path, cmd_list)

    def create_constant_speed_movements(self, tool_path):
        """
        Creates a list of axis movements that can be used to create G0 and G1 commands. If one path length is shorter
        than the other, the gantry with the shorter path length will idle at the end. This is not ideal for creating
        lofted shapes with two dissimilar paths.

        :param feed_rate_mode:
        :return:
        """
        enum = CommandLibrary.GCodeCommands
        # In Both feed rate modes the movement lengths should be the same to maintain constant speed throughout the
        # cutting path, as such, both feedrate modes are equivalent in terms of the movement commands
        self.logger.info('Creating new paths with uniform spacing between points')
        start = timeit.default_timer()
        path1c, path2c = tool_path.create_path_with_uniform_point_distances()
        self.logger.info('Took %ss to create new paths with uniform point spacing' % (timeit.default_timer() - start))

        movements = list()
        self.logger.info('Creating movement commands for XY gantry')
        start = timeit.default_timer()
        for idx in range(0, len(path1c) - 1):
            dx = path1c[idx + 1]['x'] - path1c[idx]['x']
            dy = path1c[idx + 1]['y'] - path1c[idx]['y']
            movements.append([dx, dy, 0, 0])

        self.logger.info('Creating movement commands for UZ gantry')
        for idx in range(0, len(path2c) - 1):
            du = path2c[idx + 1]['x'] - path2c[idx]['x']
            dz = path2c[idx + 1]['y'] - path2c[idx]['y']
            self.logger.info('DU: {:.6f}, DZ: {:.6f}'.format(du, dz))
            if idx < len(movements):
                movements[idx][2] = du
                movements[idx][3] = dz
            else:
                movements.append([0, 0, du, dz])

        self.logger.info('Took %ss to create movement commands' % (timeit.default_timer() - start))

        return movements, path1c[0], path2c[0]

    def create_constant_ratio_movements(self, tool_path, key_points=None):
        """
        Creates a list of axis movements that finishes each path at the same time. This allows two dissimilar shapes to
        loft properly.

        :param feed_rate_mode:
        :return:
        """
        self.logger.info('Creating new paths with equal ratio spacing between points')
        start = timeit.default_timer()
        path1c, path2c = tool_path.create_path_with_uniform_ratio_spacing(key_points)
        self.logger.info('Took %ss to create new paths with ratio spacing' % (timeit.default_timer() - start))
        debug_path = ToolPath.ToolPath(path1c, path2c)
        debug_path.plot_tool_paths()
        debug_path.plot_tool_path_connections(step=1)
        movements = list()
        self.logger.info('Creating movement commands for XY gantry')
        start = timeit.default_timer()
        for idx in range(0, len(path1c) - 1):
            dx = path1c[idx + 1]['x'] - path1c[idx]['x']
            dy = path1c[idx + 1]['y'] - path1c[idx]['y']
            movements.append([dx, dy, 0, 0])

        self.logger.info('Creating movement commands for UZ gantry')
        for idx in range(0, len(path2c) - 1):
            du = path2c[idx + 1]['x'] - path2c[idx]['x']
            dz = path2c[idx + 1]['y'] - path2c[idx]['y']
            if idx < len(movements):
                movements[idx][2] = du
                movements[idx][3] = dz
            else:
                movements.append([0, 0, du, dz])

        self.logger.info('Took %ss to create movement commands' % (timeit.default_timer() - start))

        return movements, path1c[0], path2c[0]

    def create_adaptive_ratio_movements(self, tool_path):
        enum = CommandLibrary.GCodeCommands
        pass

    def _save_gcode_file(self, file_path, cmd_list):

        with open(file_path, 'wt') as gcode_file:
            for cmd in cmd_list:
                gcode_file.write(cmd + '\n')
