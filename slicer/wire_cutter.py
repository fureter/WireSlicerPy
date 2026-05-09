import numpy as np

from util.util_functions import cmd_to_byte_array

class WireCutter(object):
    """

    """

    def __init__(self, name, wire_length, max_height, max_depth, max_speed, min_speed,
                 feed_rate_mode=94, axis_def='X{:.6f} Y{:.6f} U{:.6f} Z{:.6f}',
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
        self.max_depth = max_depth
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.feed_rate_mode = feed_rate_mode
        self.start_up_gcode = None
        self.axis_def = axis_def
        self.kerf = None
        self.max_kerf = None
        self.dynamic_tension = dynamic_tension
        self.dynamic_tension_motor_letter = None
        self.dynamic_tension_reverse = False
        self.dynamic_tension_feed_comp = False
        self.default = False

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

    def reverse_dynamic_tension(self, reverse):
        self.dynamic_tension_reverse = reverse

    def set_dynamic_tension_feed_comp(self, compensate):
        self.dynamic_tension_feed_comp = compensate


class XArm1s(object):
    HEADER = 0x55
    SERVO_MOVE = 0x03

    def __init__(self, serial_port):
        self.num_servos = 5
        self.serial = serial_port
        self.servo_positions = [0]*5
    def _servo_move_command(self, servo_id, position):
        cmd = ''.join('{:02X}'.format(a) for a in [
                              servo_id, position & 0xFF, ((position >> 8) & 0xFF)])
        return cmd

    def move_servo(self, servo_id, position, time):
        cmd = ''.join('{:02X}'.format(a) for a in [XArm1s.HEADER, XArm1s.HEADER, 8, XArm1s.SERVO_MOVE,
                                                   0x01, time & 0xFF, ((time >> 8) & 0xFF)])
        cmd += self._servo_move_command(servo_id, position)
        self.serial.write(cmd_to_byte_array(cmd))

    def move_servos(self, servo_ids, positions, time):
        cmd = ''.join('{:02X}'.format(a) for a in [XArm1s.HEADER, XArm1s.HEADER, 5+len(servo_ids)*3, XArm1s.SERVO_MOVE,
                                                   len(servo_ids), time & 0xFF, ((time >> 8) & 0xFF)])
        for (servo_id, position) in zip(servo_ids, positions):
            cmd += self._servo_move_command(servo_id, position)
        self.serial.write(cmd_to_byte_array(cmd))

class TMC2209(object):
    HEADER = 0xA0
    def __init__(self, serial_port):
        self.datagram_length =64
        self.serial = serial_port
        self.datagram = np.zeros(64, dtype=np.byte)
        self.datagram[0] = TMC2209.HEADER


    def calc_crc(self):
        crc = np.byte(0)
        for i in range(self.datagram_length-1):
            curr_byte = self.datagram[i]
            for j in range(8):
                if (crc >> 7) ^ (curr_byte & 0x01):
                    crc = (crc << 1) ^ 0x07
                else:
                    crc = crc << 1
                curr_byte = curr_byte >> 1
        return crc



