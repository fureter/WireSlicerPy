import argparse
import time

import serial
import numpy as np

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
        print(cmd)
        self.serial.write(cmd_to_byte_array(cmd))


class Stepper(object):
    HEADER = 0x55

    def __init__(self, serial_port):
        self.serial = serial_port


    def move_stepper(self, direction, angle):
        angle = int(angle * 100)
        cmd = ''.join('{:02X}'.format(a) for a in [Stepper.HEADER, Stepper.HEADER, direction &0xFF,
                                                    angle & 0xFF, ((angle >> 8) & 0xFF)])
        print(cmd)
        self.serial.write(cmd_to_byte_array(cmd))

def cmd_to_byte_array(cmd):
    packet = bytearray()
    string_list = [cmd[i:i + 2] for i in range(0, len(cmd), 2)]
    for command in string_list:
        packet.append(int(command,16))
    return packet

def parse_servo_command(line):
    servo_list = [2, 3, 4, 5, 6]
    position_list = np.zeros(5)

    splits = line.split(' ')
    curr_servo = None
    for value in splits:
        if 'P' in value:
            curr_servo = int(value[1])
        elif 'S' in value:
            position = float(value[1:])
            position_list[curr_servo - 2] = position
    return servo_list, position_list


def parse_stepper_command(line):
    splits = line.split(' ')
    for value in splits:
        if 'R' in value:
            return float(value[1:])
    return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='RobotDemoController',
        description='Runs g-code on an xArm1S and TMC2209 stepper driver.')
    parser.add_argument('-gcode', type=str, help='Filepath to the desired gcode file to be executed')
    parser.add_argument('-robot_comport', type=str, help='COMX port to send servo commands to')
    parser.add_argument('-stepper_comport', type=str, help='COMX port to send stepper commands to')

    args = vars(parser.parse_args())

    gcode_file = args['gcode']
    com_robot = args['robot_comport']
    com_stepper = args['stepper_comport']

    ser = serial.Serial(com_robot, 9600)
    ser2 = serial.Serial(com_stepper, 9600)
    xArm1s = XArm1s(ser)
    stepper = Stepper(ser2)

    travel_for_full_percent = np.deg2rad(240)
    constant_speed = np.pi/10
    previous_position_list = np.zeros(xArm1s.num_servos)

    with open(gcode_file, 'r') as f:
        for line in f:
            if len(line) > 0:
                splits = line.split(' ')
                if len(splits) > 4:
                    servo_list, position_list = parse_servo_command(line)
                    delta = np.linalg.norm(position_list - previous_position_list, ord=2)/ 1000 * travel_for_full_percent
                    move_time = delta / constant_speed
                    xArm1s.move_servos(servo_list, np.array(position_list, dtype=np.int16), time=int(move_time * 1000))
                    time.sleep(move_time)
                    previous_position_list = position_list
                else:
                    angle = parse_stepper_command(line)
                    stepper.move_stepper(direction=1, angle=angle)
                    time.sleep(1.0)





