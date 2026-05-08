import argparse
import copy

import numpy as np
import pinocchio as pin

from g_code import generator
from geometry.complex import STL, Voxel

WORKSPACE_NEUTRAL_CONFIG = np.array([0, 0, 0, 0, 0, 0])  # TODO: Based on workspace configuration, robot, and workpiece


def main():
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

    servo_command_list = []

    stl = STL(stl_file_path, units='mm')
    stl.center(ROTATION_ORIGIN)
    reference_voxel = Voxel.stl_to_voxel(cell_resolution, stl_units='mm')
    thermal_voxel = copy.deepcopy(reference_voxel).to_thermal_voxel(material=material)
    cross_sections = stl.create_theta_cross_section_pairs(stl, theta_resolution)
    reference_voxel.subtract_cross_sections_from_voxel(cross_sections, theta_resolution)

    model = pin.buildModelFromUrdf(robot_filepath)
    data = model.createData()

    # Coordinate system for the workspace is fixed and is defined by ['insert github thingy...']
    q = pin.inverse_kinematics(WORKSPACE_NEUTRAL_CONFIG)
    servo_command_list.extend(to_servo(path_plan_no_plate(q, pin.neutral(model), reference_voxel, thermal_voxel)))

    curr_theta = 0
    while reference_voxel - thermal_voxel >= tolerance:
        curr_section = cross_sections[curr_theta]
        q_list = path_plan_static_plate(q, curr_section, reference_voxel, thermal_voxel)
        servo_command_list.extend(to_servo(q_list))
        q = q_list[-1]
        curr_theta += theta_resolution

    g_code = generator.servo_command_to_gcode(servo_command_list)
    generator.save_gcode(g_code)


def to_servo(path_plan):
    """
    Converts a set of q values to servo inputs.

    :param np.ndarray path_plan: Array of robot joint commands for a given path.
    :return: List of servo inputs
    """
    return []


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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='RobotDemo',
        description='Generates g-code for a 6dof robot with a fixed thermal end-effector to cut foam. Requires an input geometry [stl] along with the robot description [pinnochio input file]. Dependencies: \n-Trimesh\n-Pinnochio\n-crocodyl')
    parser.add_argument('-stl', type=str, help='Filepath to the desired stl file to be cut')
    parser.add_argument('-robot', type=str, help='Filepath to the pinnochio compatible robot description')
    parser.add_argument('-workpiece_material', type=str, help='Filepath to the material description file')
    parser.add_argument('-wire_cutter_desc', type=str, help='Filepath to the file describing wire cutter properties')
    parser.add_argument('-out_dir', type=str, help='Filepath to the directory to save generated gcode in')
    parser.add_argument('-rod_len', type=float, help='Length of the foam cutting end effector in mm')
    parser.add_argument('-rod_radius', type=float, help='Radius of the foam cutting end effector in mm')
    parser.add_argument('-cell_resolution', type=float, default=1.0, help='Edge lengh of a single voxel cell in mm')
    parser.add_argument('-theta_resolution', type=float, default=4.0, help='Rotation resolution for cross sections in deg')
    parser.add_argument('-tolerance', type=float, default=1E-2, help='Tolerance to cut to')

    args = vars(parser.parse_args())

    ROTATION_ORIGIN = np.array([0, 0, 0])  # TODO: based on robot

    stl_file_path = args['stl']
    cell_resolution = args['cell_resolution']
    material = args['workpiece_material']
    theta_resolution = args['theta_resolution']
    robot_filepath = args['robot']
    tolerance = args['tolerance']

