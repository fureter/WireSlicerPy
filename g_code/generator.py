import datetime
import logging
import os
import sys
import timeit

import numpy as np

import g_code.command_library as command_library
from geometry.primative import GeometricFunctions
from slicer.tool_path import ToolPath
from slicer.wire_cutter import WireCutter


class TravelType():
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


class CutLayout():
    # Default cutting layout, the whole toolpath is sliced as one continuous cut.
    CONSTANT_CUT = 0
    # Cutting layout geared more towards wings and long slender cuts. Cuts the top, then bottom of the toolpath,
    # returning to the front of the cut before starting the next section.
    SPLIT_SEGMENT = 1

    RANGE = [CONSTANT_CUT, SPLIT_SEGMENT]

    @staticmethod
    def to_str(val):
        return {TravelType.CONSTANT_SPEED: 'CONSTANT_CUT',
                TravelType.CONSTANT_RATIO: 'SPLIT_SEGMENT'}[val]


class GCodeGenerator():
    def __init__(self, wire_cutter, travel_type=TravelType.CONSTANT_SPEED):
        """

        :param WireCutter wire_cutter:
        :param travel_type:
        """
        self._wire_cutter = wire_cutter
        self._travel_type = travel_type
        self.logger = logging.getLogger(__name__)

    def set_travel(self, travel_type):
        if travel_type in TravelType.RANGE:
            self._travel_type = travel_type
        else:
            raise NotImplementedError('Selected travel type of %s is not implemented' % travel_type)

    def create_relative_gcode(self, output_dir, name, tool_path, wire_cutter, part=None):
        """

        :param file_path:
        :param ToolPath tool_path:
        :param WireCutter wire_cutter:
        :return:
        """
        file_path = os.path.join(output_dir, '%s_gcode_p%s.txt' % (name, part)) if part is not None else os.path.join(
            output_dir, '%s_gcode.txt' % name)

        info_file_path = os.path.join(output_dir,
                                      '%s_info_p%s.txt' % (name, part)) if part is not None else os.path.join(
            output_dir, '%s_info.txt' % name)

        self.logger.info('Creating relative gcode with travel type: %s at location: %s' %
                         (TravelType.to_str(self._travel_type), file_path))
        cmd_list = list()
        cut_time = 0
        max_x = 0
        max_y = 0
        min_x = sys.maxsize
        min_y = sys.maxsize

        if tool_path.speed_list is None:
            speed_list = [self._wire_cutter.min_speed]
        else:
            speed_list = tool_path.speed_list
        curr_speed = speed_list[0]

        if self._wire_cutter.start_up_gcode:
            cmd_list.append(self._wire_cutter.start_up_gcode)

        enum = command_library.GCodeCommands.PositionMode
        cmd_list.append(enum.set_positioning_mode(enum.RELATIVE))

        enum = command_library.GCodeCommands.FeedRate
        cmd_list.append(enum.set_mode(self._wire_cutter.feed_rate_mode))
        cmd_list.append(enum.set_feed_rate(curr_speed))

        enum = command_library.GCodeCommands.MovementCommand

        movement_list = tool_path.get_relative_movement_list()
        posx_1 = 0
        posx_2 = 0
        posy_1 = 0
        posy_2 = 0
        prev_posx_1 = posx_1
        prev_posx_2 = posx_2
        prev_posy_1 = posy_1
        prev_posy_2 = posy_2
        for ind in range(0, len(movement_list)):
            if len(speed_list) > 1:
                if curr_speed != speed_list[ind]:
                    curr_speed = speed_list[ind]
                    cmd_list.append(command_library.GCodeCommands.FeedRate.set_feed_rate(curr_speed))
                    self.logger.debug(command_library.GCodeCommands.FeedRate.set_feed_rate(curr_speed))

            movement = movement_list[ind]

            # Calculate the relative change in the wire length between the two gantries
            posx_1 += movement[0]
            posx_2 += movement[2]
            posy_1 += movement[1]
            posy_2 += movement[3]
            if wire_cutter.dynamic_tension:

                # Wire length adjustments account for the change in the wire length between each gantry, plus the change
                # in the wire length between the gantry head and the tensioning motor.
                # TODO: Assumes the tensioning motor is mounted above the XY Gantry, replace first dY with whichever
                #  gantry has the tensioning (e.g. dZ)
                # Equation below is from taking the derivative of the wire length wrt a discrete 'interval'
                # dL = -dY + (Lx(dX-dU) + Ly(dY-dZ))/Lc
                # Where Lc is the separation between the gantrys (cutting length)
                lx = posx_1 - posx_2
                ly = posy_1 - posy_2
                lx_prev = prev_posx_1 - prev_posx_2
                ly_prev = prev_posy_1 - prev_posy_2
                length_curr = np.sqrt(wire_cutter.wire_length**2 + lx**2 + ly**2)
                length_prev = np.sqrt(wire_cutter.wire_length**2 + lx_prev**2 + ly_prev**2)
                delta_wire = -movement[1] + (length_curr - length_prev)
                if wire_cutter.dynamic_tension_reverse:
                    delta_wire *= -1
                tension_comp = ' {motor}{dist:.6f}'.format(
                    motor=wire_cutter.dynamic_tension_motor_letter,
                    dist=delta_wire)
                dist = np.sqrt(delta_wire ** 2 + movement[0]**2 + movement[1]**2 + movement[2]**2 + movement[3]**2)
                if wire_cutter.dynamic_tension_feed_comp:
                    v = abs(delta_wire)
                    c = abs(movement[0]) + abs(movement[1]) + abs(movement[2]) + abs(movement[3])
                    feed_rate = (1 + v/c)*curr_speed if c != 0 else curr_speed
                    feed_comp = ' F{feed:.3f}'.format(feed=feed_rate)
                else:
                    feed_rate = curr_speed
                    feed_comp = ''

            else:
                tension_comp = ''
                feed_rate = curr_speed
                feed_comp = ''
                dist = np.sqrt(movement[0]**2 + movement[1]**2 + movement[2]**2 + movement[3]**2)

            if max(posx_2, posx_1) > max_x:
                max_x = max(posx_2, posx_1)
            if max(posy_2, posy_1) > max_y:
                max_y = max(posy_2, posy_1)
            if min(posx_2, posx_1) < min_x:
                min_x = min(posx_2, posx_1)
            if min(posy_2, posy_1) < min_y:
                min_y = min(posy_2, posy_1)

            cmd_list.append(enum.g1_linear_move(
                self._wire_cutter.axis_def.format(movement[0], movement[1], movement[2],
                                                  movement[3]) + tension_comp + feed_comp))
            cut_time += (dist / feed_rate) * 60.0  # [mm] / [mm/min] * [s/min] => [s]

            prev_posx_1 = posx_1
            prev_posx_2 = posx_2
            prev_posy_1 = posy_1
            prev_posy_2 = posy_2

        self._save_info_file(info_file_path, cut_time, max_x, max_y, min_x, min_y)
        self._save_gcode_file(file_path, cmd_list)

    def create_absolute_gcode(self, file_path, tool_path):
        """

        :param file_path:
        :param ToolPath tool_path:
        :param list[Point] key_points:
        :param int cut_mode: Defines how the toolpath should be sliced.
        :return:
        """
        self.logger.info('Creating absolute gcode with travel type: %s at location: %s' %
                         (TravelType.to_str(self._travel_type), file_path))
        cmd_list = list()

        if self._wire_cutter.start_up_gcode:
            cmd_list.extend(self._wire_cutter.start_up_gcode)

        enum = command_library.GCodeCommands.PositionMode
        cmd_list.append(enum.set_positioning_mode(enum.ABSOLUTE))

        enum = command_library.GCodeCommands.FeedRate
        cmd_list.append(enum.set_mode(self._wire_cutter.feed_rate_mode))
        cmd_list.append(enum.set_feed_rate(self._wire_cutter.min_speed))

        enum = command_library.GCodeCommands.MovementCommand

        movement_list = tool_path.get_absolute_movement_list()

        for movement in movement_list:
            cmd_list.append(enum.g1_linear_move(
                self._wire_cutter.axis_def.format(movement[0], movement[1], movement[2], movement[3])))

        self._save_gcode_file(file_path, cmd_list)

    def create_constant_speed_movements(self, tool_path, cut_mode=CutLayout.CONSTANT_CUT):
        """
        Creates a list of axis movements that can be used to create G0 and G1 commands. If one path length is shorter
        than the other, the gantry with the shorter path length will idle at the end. This is not ideal for creating
        lofted shapes with two dissimilar paths.

        :param feed_rate_mode:
        :return:
        """
        enum = command_library.GCodeCommands
        # In Both feed rate modes the movement lengths should be the same to maintain constant speed throughout the
        # cutting path, as such, both feedrate modes are equivalent in terms of the movement commands
        self.logger.info('Creating new paths with uniform spacing between points')
        start = timeit.default_timer()
        path1c, path2c = tool_path.create_path_with_uniform_point_distances()
        self.logger.info('Took %ss to create new paths with uniform point spacing' % (timeit.default_timer() - start))

        movements = [list(), list()]
        self.logger.info('Creating movement commands for XY gantry')
        start = timeit.default_timer()
        for idx in range(0, len(path1c) - 1):
            dx = path1c[idx + 1]['x'] - path1c[idx]['x']
            dy = path1c[idx + 1]['y'] - path1c[idx]['y']
            movements[0].append([dx, dy, 0, 0])

        self.logger.info('Creating movement commands for UZ gantry')
        for idx in range(0, len(path2c) - 1):
            du = path2c[idx + 1]['x'] - path2c[idx]['x']
            dz = path2c[idx + 1]['y'] - path2c[idx]['y']
            self.logger.info('DU: {:.6f}, DZ: {:.6f}'.format(du, dz))
            if idx < len(movements[0]):
                movements[0][idx][2] = du
                movements[0][idx][3] = dz
            else:
                movements[0].append([0, 0, du, dz])

        self.logger.info('Took %ss to create movement commands' % (timeit.default_timer() - start))

        return movements, path1c[0], path2c[0]

    def create_constant_ratio_movements(self, tool_path, key_points=None, cut_mode=CutLayout.CONSTANT_CUT):
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
        debug_path = ToolPath(path1c, path2c)
        debug_path.plot_tool_paths()
        debug_path.plot_tool_path_connections(step=1)

        if cut_mode is CutLayout.SPLIT_SEGMENT:
            max_idx_x = GeometricFunctions.get_index_max_coord(path1c, 'x')
            max_idx_u = GeometricFunctions.get_index_max_coord(path2c, 'x')
        else:
            max_idx_x = len(path1c) - 1
            max_idx_u = len(path2c) - 1

        movements = [list(), list()]
        self.logger.info('Creating movement commands for XY gantry')
        start = timeit.default_timer()
        for idx in range(0, max_idx_x - 1):
            dx = path1c[idx + 1]['x'] - path1c[idx]['x']
            dy = path1c[idx + 1]['y'] - path1c[idx]['y']
            movements[0].append([dx, dy, 0, 0])

        if cut_mode is CutLayout.SPLIT_SEGMENT:
            for idx in reversed(range(max_idx_x, len(path1c) - 1)):
                dx = path1c[idx]['x'] - path1c[idx+1]['x']
                dy = path1c[idx]['y'] - path1c[idx+1]['y']
                movements[1].append([dx, dy, 0, 0])

        self.logger.info('Creating movement commands for UZ gantry')
        for idx in range(0, max_idx_u - 1):
            du = path2c[idx + 1]['x'] - path2c[idx]['x']
            dz = path2c[idx + 1]['y'] - path2c[idx]['y']
            if idx < len(movements[0]):
                movements[0][idx][2] = du
                movements[0][idx][3] = dz
            else:
                movements[0].append([0, 0, du, dz])

        if cut_mode is CutLayout.SPLIT_SEGMENT:
            i = 0
            for idx in reversed(range(max_idx_u, len(path2c)-1)):
                du = path2c[idx]['x'] - path2c[idx+1]['x']
                dz = path2c[idx]['y'] - path2c[idx+1]['y']
                if i < len(movements[1]):
                    movements[1][i][2] = du
                    movements[1][i][3] = dz
                    i += 1
                else:
                    movements[1].append([0, 0, du, dz])

        self.logger.info('Took %ss to create movement commands' % (timeit.default_timer() - start))

        return movements, path1c[0], path2c[0]

    def create_adaptive_ratio_movements(self, tool_path):
        enum = command_library.GCodeCommands
        pass

    def _save_gcode_file(self, file_path, cmd_list):

        with open(file_path, 'wt') as gcode_file:
            for cmd in cmd_list:
                gcode_file.write(cmd + '\n')

    def _save_info_file(self, file_path, cut_time, max_x, max_y, min_x, min_y):
        with open(file_path, 'wt') as info_file:
            info_file.write('Estimated cut time: %s\n' % str(datetime.timedelta(seconds=cut_time)))
            info_file.write('Max X position: %smm\n' % max_x)
            info_file.write('Max Y position: %smm\n' % max_y)
            info_file.write('Min X position: %smm\n' % min_x)
            info_file.write('Min Y position: %smm\n' % min_y)
            info_file.write('Total X span: %smm\n' % (max_x - min_x))
            info_file.write('Total Y span: %smm\n' % (max_y - min_y))
