import argparse
import copy
import os
from pathlib import Path

import serial
import numpy as np
import pinocchio as pin

import geometry.spatial_manipulation
from g_code import generator
from geometry.complex import STL, Voxel, ThermalVoxel, CrossSection
from geometry.primative import Plane
from geometry.primative import Path as primPath
from geometry.spatial_manipulation import PointManip
from slicer.materials import Material
from assets.Robot.xArm1S import xArm1s

WORKSPACE_NEUTRAL_CONFIG = np.array([0, 0, 0, 0, 0, 0])  # TODO: Based on workspace configuration, robot, and workpiece


def main(stl_file_path, robot_file_path, robot_comport, motor_comport, workpiece_material, wire_cutter_desc, output_dir, cell_resolution,
         theta_resolution, tolerance, live_control, debug):
    """
    Algorithm outline:
    1. Load Geometry
    2. Discretize as voxel grid
    3. Initilize Material properites in voxel grid
    4. Generate 2D contour paths at [theta] grid points
    5. Generate reference voxel grid from 2D contour paths from voxel grid
    6. Load Robot parameters
    7. Move robot to initialization location while avoiding voxel grid
    8. While Voxel grid <= Reference Voxel grid:
        Generate cut paths using crocodyl and subtract simulate them through the voxel grid
    9. When Voxel grid meets resolution spec:
        Generate g-code file from servo position commands, save to output directory

    This approach currently assumes a constant electrical input to the cutting tool and does not generate any power
    supply commands. Electrical and thermal constants need to be supplied to the program for melt-front modeling.
    """
    base_height = 20
    top_height = 40
    if live_control:
        robot_com = serial.Serial(robot_comport, 9600)
        motor_com = serial.Serial(robot_comport, 9600)

    robot_object = None
    if live_control:
        if 'xarm1s'.lower() in robot_file_path.lower():
            robot_object = xArm1s.XArm1s(robot_com)
        else:
            raise NotImplementedError('Robot %s is not implemented' % robot_file_path)

    debug_dir = None
    if debug:
        Path(os.path.join(output_dir, 'debug')).mkdir(exist_ok=True)
        debug_dir = os.path.join(output_dir, 'debug')

    material = Material(workpiece_material)
    servo_command_list = []
    ROTATION_ORIGIN = np.array([0.40, 0.0, 0.0])*1000  # Millimeters

    stl = STL(stl_file_path, units='mm')
    stl.mesh.apply_scale(1.0)
    stl.center(ROTATION_ORIGIN)
    reference_voxel = Voxel.stl_to_voxel(stl, cell_resolution)
    reference_voxel = ThermalVoxel(reference_voxel, cell_resolution, material=material)

    xlen, ylen, zlen = 100.0, 100.0, 200.0
    xnum, ynum, znum = np.int32(np.array([xlen, ylen, zlen])/cell_resolution)
    work_voxel = ThermalVoxel(data=np.ones([xnum,ynum,znum]), resolution=cell_resolution, material=material)

    origin_plane = Plane.plane_from_point_norm(ROTATION_ORIGIN, np.array([1.0, 0.0, 0.0]))
    stl.slice_into_theta_cross_sections(origin_plane, theta_resolution, output_dir=debug_dir, num_points=512)
    cross_sections = stl.cross_sections
    cross_sections = add_base_and_top_cut(cross_sections, base_height, top_height)

    # Load the robot data
    model = pin.buildModelFromUrdf(robot_file_path)
    data = model.createData()

    # Coordinate system for the workspace is fixed and is defined by ['insert github thingy...']
    q = pin.inverse_kinematics(WORKSPACE_NEUTRAL_CONFIG)
    servo_command_list.extend(robot_object.to_servo(np.array(q)))

    if live_control:
        pass
        #move robot

    curr_theta = 0
    while reference_voxel - work_voxel >= tolerance:
        curr_section = cross_sections[curr_theta]
        q_list = path_plan_static_plate(q, curr_section, reference_voxel, work_voxel)
        servo_command_list.extend(robot_object.to_servo(q_list))
        q = q_list[-1]
        curr_theta += 1
        servo_command_list.extend(np.array(theta_resolution))
        if live_control:
            pass
            # Move robot
            # Move rotation plate

    stl_name = stl_file_path.split(os.sep)[-1].split('.')[0]
    robot_name = robot_file_path.split(os.sep)[-1].split('.')[0]

    g_code = generator.servo_command_to_gcode(servo_command_list)
    generator.save_gcode_file(os.path.join(output_dir, '%s_%s_%sdeg_gcode.txt' % (stl_name, robot_name, theta_resolution)), g_code)


def path_plan_no_plate(q_target, q_start, reference_voxel, thermal_voxel):
    """
    Generate a motion path of robot joint inputs to get from q_start to q_target with a cost function of avoiding the
    workpiece and any collisions.

    :param q_target:
    :param q_start:
    :param reference_voxel:
    :param thermal_voxel:
    :return:
    """
    return []


def path_plan_static_plate(q_start, curr_section, reference_voxel, thermal_voxel):
    """
    Generate a motion path of robot joint inputs to move anywhere in the workspace with the cost function of maximizing
    the removal of material from thermal_voxel with respect ot reference_voxel.

    :param q_start:
    :param curr_section:
    :param reference_voxel:
    :param thermal_voxel:
    :return:
    """
    return []

def add_base_and_top_cut(cross_sections, base_height, top_height):
    new_sections = list()
    for section in cross_sections:
        section_path = copy.deepcopy(section.section_list[0])
        PointManip.Transform.translate(section_path, np.array([0.0, 0.0, base_height]))
        path_start = copy.deepcopy(section_path[0])
        base_path = list()
        for i in range(32):
            new_point = copy.deepcopy(path_start)
            new_point[1] += (32-i)/32.0 * 30
            base_path.append(copy.deepcopy(new_point))
        base_path.extend(section_path)
        section_path = copy.deepcopy(base_path)
        path_end = copy.deepcopy(section_path[-1])
        for i in range(1, 33):
            new_point = copy.deepcopy(path_end)
            new_point[2] += (i)/32.0 * top_height
            section_path.append(copy.deepcopy(new_point))
        new_sections.append(CrossSection([primPath(section_path)]))
    return new_sections





if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='RobotDemo',
        description='Generates g-code for a 6dof robot with a fixed thermal end-effector to cut foam. Requires an input geometry [stl] along with the robot description [pinnochio input file]. Dependencies: \n-Trimesh\n-Pinnochio\n-crocodyl')
    parser.add_argument('-stl', type=str, help='Filepath to the desired stl file to be cut')
    parser.add_argument('-robot', type=str, help='Filepath to the pinnochio compatible robot description')
    parser.add_argument('-workpiece_material', type=str, help='Filepath to the material description file')
    parser.add_argument('-wire_cutter_desc', type=str, help='Filepath to the file describing wire cutter properties')
    parser.add_argument('-out_dir', type=str, help='Filepath to the directory to save generated gcode in')
    parser.add_argument('-cell_resolution', type=float, default=1.0, help='Edge length of a single voxel cell in mm')
    parser.add_argument('-theta_resolution', type=float, default=10.0, help='Rotation resolution for cross sections in deg')
    parser.add_argument('-tolerance', type=float, default=1E-2, help='Tolerance to cut to')
    parser.add_argument('-live_control', type=bool, default=False, help='If True will use supplied com ports to send live control commands')
    parser.add_argument('-robot_comport', type=str, help='COM port to send robot commands on')
    parser.add_argument('-motor_comport', type=str, help='COM port to send motor rotation commands on')
    parser.add_argument('--debug', action='store_true', help='If enabled will create a debug directory in -out_dir and save intermediate steps')

    args = vars(parser.parse_args())

    stl_file_path = args['stl']
    robot_file_path = args['robot']
    robot_comport = args['robot_comport']
    motor_comport = args['motor_comport']
    workpiece_material = args['workpiece_material']
    wire_cutter_desc = args['wire_cutter_desc']
    output_dir = args['out_dir']
    cell_resolution = args['cell_resolution']
    material = args['workpiece_material']
    theta_resolution = args['theta_resolution']
    robot_filepath = args['robot']
    tolerance = args['tolerance']
    live_control = args['live_control']
    debug = args['debug']

    main(stl_file_path, robot_file_path, robot_comport, motor_comport, workpiece_material, wire_cutter_desc, output_dir, cell_resolution,
         theta_resolution, tolerance, live_control, debug)

