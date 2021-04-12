from Slicer.WireCutter import WireCutter
from Slicer.ToolPath import ToolPath
from GCode.CommandLibrary import GCodeCommands

class TravelType(object):
    # Maintains both gantry's at a constant speed.
    CONSTANT_SPEED = 0
    # Maintains a constant speed/total_distance between both gantries
    CONSTANT_RATIO = 1
    # Maintains a constant speed/total_distance between both gantries per toolpath
    ADAPTIVE_RATIO = 2

    RANGE = [CONSTANT_SPEED, CONSTANT_RATIO, ADAPTIVE_RATIO]

    @classmethod
    def to_str(cls, val):
        return {cls.CONSTANT_SPEED: 'CONST_SPEED',
                cls.CONSTANT_RATIO: 'CONST_RATIO',
                cls.ADAPTIVE_RATIO: 'ADAPT_RATIO'}[val]


class GCodeGenerator(object):
    def __init__(self, wire_cutter, travel_type=TravelType.CONSTANT_SPEED):
        """

        :param WireCutter wire_cutter:
        :param travel_type:
        """
        self._wire_cutter = wire_cutter
        self._travel_type = travel_type

    def set_travel(self, travel_type):
        if travel_type in TravelType.RANGE:
            self._travel_type = travel_type
        else:
            raise NotImplementedError('Selected travel type of %s is not implemented' % travel_type)

    def create_relative_gcode(self, file_path, tool_path):
        """

        :param file_path:
        :param ToolPath tool_path:
        :return:
        """
        cmd_list = list()

        if self._wire_cutter.start_up_gcode:
            cmd_list.extend(self._wire_cutter.start_up_gcode)

        enum = GCodeCommands.PositionMode
        cmd_list.append(enum.set_positioning_mode(enum.RELATIVE))

        if self._travel_type == TravelType.CONSTANT_SPEED:
            movement_list = tool_path.create_constant_speed_toolpath(self._wire_cutter.feed_rate_mode)
        elif self._travel_type == TravelType.CONSTANT_RATIO:
            movement_list = tool_path.create_constant_ratio_toolpath(self._wire_cutter.feed_rate_mode)
        elif self._travel_type == TravelType.ADAPTIVE_RATIO:
            movement_list = tool_path.create_adaptive_ratio_toolpath(self._wire_cutter.feed_rate_mode)
        else:
            raise ValueError('Travel Type of %s is invalid' % self._travel_type)

        enum = GCodeCommands.MovementCommand
        for movement in movement_list:
            cmd_list.append(enum.g1_linear_move(self._wire_cutter.axis_def % movement))

        self._save_gcode_file(file_path, cmd_list)

    def _save_gcode_file(self, file_path, cmd_list):

        with open(file_path) as gcode_file:
            for cmd in cmd_list:
                gcode_file.write(cmd + '\r\n')
