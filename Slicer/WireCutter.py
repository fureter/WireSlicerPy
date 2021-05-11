class WireCutter(object):
    """

    """

    def __init__(self, wire_length, max_height, max_speed, min_speed, release_height=None, start_height=None,
                 start_depth=None, feed_rate_mode=0,#GCodeCommands.FeedRate.UNITS_PER_MINUTE,
                 axis_def='X%s Y%s U%s Z%s'):
        """

        :param wire_length:
        :param max_height:
        :param max_speed:
        :param min_speed:
        :param release_height:
        :param start_height:
        """
        self.wire_length = wire_length
        self.max_height = max_height
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.release_height = release_height
        self.start_height = start_height
        self.start_depth = start_depth
        self.feed_rate_mode = feed_rate_mode
        self.start_up_gcode = None
        self.axis_def = axis_def

    def set_gcode_statup(self, g_code):
        """Sets the initializing g_code for the machine.

        :param g_code: Initial g_code entries at the beginning of generated g_code file.
        :type g_code: list[str].
        """
        self.start_up_gcode = g_code
