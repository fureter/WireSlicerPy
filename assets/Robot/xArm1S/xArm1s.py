
class XArm1s(object):
    HEADER = 0x55
    SERVO_MOVE = 0x03
    SERVO_READ = 0x04
    SERVO_WRITE = 0x05

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
        self.serial.write(self.cmd_to_byte_array(cmd))

    def move_servos(self, servo_ids, positions, time):
        cmd = ''.join('{:02X}'.format(a) for a in [XArm1s.HEADER, XArm1s.HEADER, 5+len(servo_ids)*3, XArm1s.SERVO_MOVE,
                                                   len(servo_ids), time & 0xFF, ((time >> 8) & 0xFF)])
        for (servo_id, position) in zip(servo_ids, positions):
            cmd += self._servo_move_command(servo_id, position)
        self.serial.write(self.cmd_to_byte_array(cmd))

    def to_servo(self, path_plan, robot):
        """
        Converts a set of q values to servo inputs.

        :param np.ndarray path_plan: Array of robot joint commands for a given path.
        :param pin.Model robot: Robot model data, used to get joint limits.
        :return: List of servo inputs
        """
        servo_commands = []
        for q in path_plan:
            perc_travel = q / robot.model.limits
            servo_commands.append(perc_travel * 500 + 500)
        return servo_commands

    @staticmethod
    def cmd_to_byte_array(cmd):
        packet = bytearray()
        string_list = [cmd[i:i + 2] for i in range(0, len(cmd), 2)]
        for command in string_list:
            packet.append(int(command, 16))
        return packet