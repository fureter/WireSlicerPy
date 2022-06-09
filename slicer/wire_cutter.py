class WireCutter(object):
    """

    """

    def __init__(self, name, wire_length, max_height, max_speed, min_speed, release_height=None, start_height=None,
                 start_depth=None, feed_rate_mode=94, axis_def='X{:.6f} Y{:.6f} U{:.6f} Z{:.6f}',
                 dynamic_tension=False):
        """

        :param wire_length:
        :param max_height:
        :param max_speed:
        :param min_speed:
        :param release_height:
        :param start_height:
        """
        self.name = name
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
        self.kerf = None
        self.dynamic_tension = dynamic_tension
        self.dynamic_tension_motor_letter = None
        self.dynamic_tension_spool_radius = None
        self.reverse = False

    def set_gcode_statup(self, g_code):
        """Sets the initializing test_g_code for the machine.

        :param g_code: Initial test_g_code entries at the beginning of generated test_g_code file.
        :type g_code: list[str].
        """
        self.start_up_gcode = g_code

    def set_kerf(self, kerf, max_kerf):
        """

        :param kerf: Generalized kerf value in mm^2. Kerf value is calculated as kerf/path_length.
        :param max_kerf: Band-aid for handling very small sections.
        """
        self.kerf = kerf
        self.max_kerf = max_kerf

    def set_dynamic_tension_motor_letter(self, motor_letter):
        self.dynamic_tension_motor_letter = motor_letter

    def set_dynamic_tension_spool_radius(self, spool_radius):
        self.dynamic_tension_spool_radius = spool_radius

    def reverse_dynamic_tension(self, reverse):
        self.reverse = reverse
