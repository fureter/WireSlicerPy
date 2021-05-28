import timeit

import g_code.command_library as command_library
from geometry.primative import GeometricFunctions
from slicer.wire_cutter import WireCutter
from slicer.tool_path import ToolPath


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

    # TODO: This code was not thought out well for future feature addition. Needs to be made more abstract to support
    #  multiple toolpath links (Segment Cut for example which was just slapped in below, or spars)
    #  The cut_mode and linking of toolpaths really belong in the test_slicer sub-package, Generator should only really
    #  be handling taking the toolpath and turning it into test_g_code for the given wire_cutter
    def create_relative_gcode(self, file_path, tool_path, key_points=None, cut_mode=CutLayout.CONSTANT_CUT):
        """

        :param file_path:
        :param tool_path tool_path:
        :param list[Point] key_points:
        :param int cut_mode: Defines how the toolpath should be sliced.
        :return:
        """
        self.logger.info('Creating relative gcode with travel type: %s at location: %s' %
                         (TravelType.to_str(self._travel_type), file_path))
        cmd_list = list()
        # TODO: This really should've been planned out better, working in absolute coordinates is so much easier, with
        #  relative coordinates every move should've been planned out in advance, not adding movements in the
        #  moment
        curr_x = 0
        curr_y = 0
        curr_u = 0
        curr_z = 0

        if self._wire_cutter.start_up_gcode:
            cmd_list.extend(self._wire_cutter.start_up_gcode)

        enum = command_library.GCodeCommands.PositionMode
        cmd_list.append(enum.set_positioning_mode(enum.RELATIVE))

        enum = command_library.GCodeCommands.FeedRate
        cmd_list.append(enum.set_mode(self._wire_cutter.feed_rate_mode))
        cmd_list.append(enum.set_feed_rate(self._wire_cutter.min_speed))

        enum = command_library.GCodeCommands.MovementCommand
        start_height = self._wire_cutter.start_height
        start_depth = self._wire_cutter.start_depth
        cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(0.0, start_height, 0.0, start_height)))
        cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_depth, 0.0, start_depth, 0.0)))
        curr_x += start_depth
        curr_u += start_depth
        curr_y += start_height
        curr_z += start_height

        if self._travel_type == TravelType.CONSTANT_SPEED:
            movement_list, start_point1, start_point2 = self.create_constant_speed_movements(tool_path, cut_mode)
        elif self._travel_type == TravelType.CONSTANT_RATIO:
            movement_list, start_point1, start_point2 = self.create_constant_ratio_movements(tool_path, key_points,
                                                                                             cut_mode)
        elif self._travel_type == TravelType.ADAPTIVE_RATIO:
            raise NotImplementedError('Adaptive Ratio is not currently implemented')
        else:
            raise ValueError('Travel Type of %s is invalid' % self._travel_type)

        cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_point1['x'], start_point1['y'],
                                                                              start_point2['x'], start_point2['y'])))
        curr_x += start_point1['x']
        curr_u += start_point2['x']
        curr_y += start_point1['y']
        curr_z += start_point2['y']

        for movement in movement_list[0]:
            cmd_list.append(enum.g1_linear_move(
                self._wire_cutter.axis_def.format(movement[0], movement[1], movement[2], movement[3])))
            curr_x += movement[0]
            curr_u += movement[2]
            curr_y += movement[1]
            curr_z += movement[3]

        if cut_mode is CutLayout.SPLIT_SEGMENT and len(movement_list) == 2:
            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_depth, 0.0, start_depth, 0.0)))
            curr_x += start_depth
            curr_u += start_depth

            release_height = self._wire_cutter.release_height
            # If the cut mode is Split Segment then two movement lists should've been returned by the above creator
            # functions. Move the wire back to the start of the cut and do the second cut path.
            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(
                0, release_height, 0, release_height)))
            curr_y += release_height
            curr_z += release_height

            cmd_list.append(command_library.GCodeCommands.FeedRate.set_feed_rate(self._wire_cutter.max_speed))

            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(
                -curr_x, 0, -curr_u, 0)))
            curr_x -= curr_x
            curr_u -= curr_u

            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(
                0, -curr_y + start_height, 0, -curr_z + start_height)))
            curr_y -= curr_y + start_height
            curr_z -= curr_z + start_height

            cmd_list.append(command_library.GCodeCommands.FeedRate.set_feed_rate(self._wire_cutter.min_speed))

            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_depth, 0.0, start_depth, 0.0)))
            curr_x += start_depth
            curr_u += start_depth

            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_point1['x'],
                                                                                  start_point1['y'],
                                                                                  start_point2['x'],
                                                                                  start_point2['y'])))
            curr_x += start_point1['x']
            curr_u += start_point2['x']
            curr_y += start_point1['y']
            curr_z += start_point2['y']

            for movement in movement_list[1]:
                cmd_list.append(enum.g1_linear_move(
                    self._wire_cutter.axis_def.format(movement[0], movement[1], movement[2], movement[3])))
                curr_x += movement[0]
                curr_u += movement[2]
                curr_y += movement[1]
                curr_z += movement[3]

            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(start_depth, 0.0, start_depth, 0.0)))
            curr_x += start_depth
            curr_u += start_depth

            # If the cut mode is Split Segment then two movement lists should've been returned by the above creator
            # functions. Move the wire back to the start of the cut and do the second cut path.
            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(
                0, release_height, 0, release_height)))
            curr_y += release_height
            curr_z += release_height

            cmd_list.append(command_library.GCodeCommands.FeedRate.set_feed_rate(self._wire_cutter.max_speed))

            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(
                -curr_x, 0, -curr_u, 0)))
            curr_x -= curr_x
            curr_u -= curr_u

            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def.format(
                0, -curr_y + start_height, 0, -curr_z + start_height)))
            curr_y -= curr_y + start_height
            curr_z -= curr_z + start_height

        else:
            # If cut_mode is Constant_cut then the cut is already complete, move the wire out of the work piece.
            cmd_list.append(enum.g1_linear_move(
                self._wire_cutter.axis_def.format(-start_depth, 0.0, -start_depth, 0.0)))
            curr_x -= start_depth
            curr_u -= start_depth

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
            for idx in reversed(range(max_idx_u, len(path2c)-1)):
                du = path2c[idx]['x'] - path2c[idx+1]['x']
                dz = path2c[idx]['y'] - path2c[idx+1]['y']
                if idx < len(movements[0]):
                    movements[1][idx][2] = du
                    movements[1][idx][3] = dz
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
