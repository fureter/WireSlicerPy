import argparse
import copy
import os
import time
from pathlib import Path

import meshcat_shapes
import meshcat.geometry as g
import meshcat.transformations as tf
import serial
import numpy as np
import pinocchio as pin
from matplotlib import pyplot as plt
from pinocchio.visualize import MeshcatVisualizer
from qpsolvers import solve_qp

import geometry.spatial_manipulation
from g_code import generator
from geometry.complex import STL, Voxel, ThermalVoxel, CrossSection
from geometry.primative import Plane
from geometry.primative import Path as primPath
from geometry.primative import Point
from geometry.primative import GeometricFunctions
from geometry.spatial_manipulation import PointManip
from slicer.materials import Material
from assets.Robot.xArm1S import xArm1s
from slicer.wire_cutter import CuttingTool

WORKSPACE_NEUTRAL_CONFIG = np.array([0, 0, 0, 0, 0, 0])  # TODO: Based on workspace configuration, robot, and workpiece


def main(stl_file_path, robot_file_path, workpiece_material, wire_cutter_desc, output_dir, cell_resolution,
         theta_resolution, tolerance, debug):
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
    output_dir = output_dir.replace('\\', os.sep)
    base_height = 0.02 + 0.08  # 8cm from robot anchor origin to end of the needles holding the foam in place
    top_height = 0.04

    debug_dir = None
    if debug:
        Path(os.path.join(output_dir, 'debug')).mkdir(exist_ok=True)
        debug_dir = os.path.join(output_dir, 'debug')

    if workpiece_material is not None:
        workpiece_material = Material(workpiece_material.replace('\\', os.sep))
    if wire_cutter_desc is not None:
        cutting_tool = CuttingTool(wire_cutter_desc.replace('\\', os.sep))
    servo_command_list = []
    ROTATION_ORIGIN = np.array([0.248, 0.0, base_height]) * 1000  # Millimeters
    stl_file_path = stl_file_path.replace('\\', os.sep)
    stl = STL(stl_file_path, units='mm')
    stl.mesh.apply_scale(2.5)
    stl.center(ROTATION_ORIGIN)
    reference_voxel = Voxel.stl_to_voxel(stl, cell_resolution)
    if workpiece_material is not None:
        reference_voxel = ThermalVoxel(reference_voxel, cell_resolution, material=workpiece_material)

    xlen, ylen, zlen = 100.0, 100.0, 200.0
    xnum, ynum, znum = np.int32(np.array([xlen, ylen, zlen]) / cell_resolution)

    if workpiece_material is not None:
        work_voxel = ThermalVoxel(data=np.ones([xnum, ynum, znum]), resolution=cell_resolution,
                                  material=workpiece_material)
    else:
        work_voxel = None

    origin_plane = Plane.plane_from_point_norm(ROTATION_ORIGIN, np.array([1.0, 0.0, 0.0]))
    stl.slice_into_theta_cross_sections(origin_plane, theta_resolution, base_height, output_dir=debug_dir,
                                        num_points=128)
    cross_sections = stl.cross_sections

    if workpiece_material is not None and wire_cutter_desc is not None:
        thermal_cross_sections = compensate_for_melt_front(cross_sections, workpiece_material, cutting_tool)
        if debug:
            theta = 0
            for cross_section, thermal_cross in zip(cross_sections, thermal_cross_sections):
                plt.figure(figsize=(8, 4.5))
                cross_section.plot(axis1='x', axis2='z', scatter=True)
                thermal_cross.plot(axis1='x', axis2='z', scatter=True)
                plt.grid(True)
                plt.axis('equal')
                plt.savefig(os.path.join(output_dir, 'debug', "cross_section_thermal_comp_theta%s.png" % theta))
                theta += 1
                plt.close()
        cross_sections = thermal_cross_sections

    cross_sections = add_base_and_top_cut(cross_sections, top_height)

    # Load the robot data
    robot_file_path = robot_file_path.replace('\\', os.sep)
    robot = pin.RobotWrapper.BuildFromURDF(
        filename=robot_file_path
    )

    if debug:
        # Create a visualizer
        vis = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
        robot.setVisualizer(vis, init=True)
        vis.initViewer(open=True)
        vis.loadViewerModel()

        # Choose what to display
        vis.displayFrames(False)
        vis.displayVisuals(True)
        vis.displayCollisions(False)

        vis.viewer["mesh"].set_object(g.TriangularMeshGeometry(
            vertices=stl.mesh.vertices / 1000.0,  # Convert mm to m
            faces=stl.mesh.faces
        ))

    q = pin.neutral(robot.model)
    # - Compute the placement of all joint frames
    pin.forwardKinematics(robot.model, robot.data, q)
    # - Compute the placement of all link frames
    pin.updateFramePlacements(robot.model, robot.data)
    if debug:
        vis.display(q)
        for frame in robot.model.frames:
            meshcat_shapes.frame(vis.viewer['frames/' + frame.name], opacity=0.75, axis_length=0.05)
            frame_id = robot.model.getFrameId(frame.name)
            vis.viewer['frames/' + frame.name].set_transform(robot.data.oMf[frame_id].homogeneous)
    servo_command_list.extend(to_servo(np.array(q), robot))

    reverse_path = False
    for curr_theta in range(len(cross_sections)):
        print('Processing theta %d Cross Section' % (curr_theta * theta_resolution))
        curr_section = cross_sections[curr_theta]
        q_list = path_plan_static_plate(q, curr_section, reference_voxel, work_voxel, robot, reverse_path)
        reverse_path = not reverse_path
        servo_command_list.extend(to_servo(q_list, robot))
        q = q_list[-1]
        curr_theta += 1
        servo_command_list.append(np.array([theta_resolution]))
        if debug:
            for q_vis in q_list:
                vis.display(q_vis)
                for frame in robot.model.frames:
                    frame_id = robot.model.getFrameId(frame.name)
                    vis.viewer['frames/' + frame.name].set_transform(robot.data.oMf[frame_id].homogeneous)
                time.sleep(0.05)

    stl_name = stl_file_path.split(os.sep)[-1].split('.')[0]
    robot_name = robot_file_path.split(os.sep)[-1].split('.')[0]

    g_code = generator.servo_command_to_gcode(servo_command_list)
    generator.save_gcode_file(
        os.path.join(output_dir, '%s_%s_%sdeg_gcode.txt' % (stl_name, robot_name, theta_resolution)), g_code)


def to_servo(path_plan, robot):
    """
    Converts a set of q values to servo inputs.

    :param np.ndarray path_plan: Array of robot joint commands for a given path.
    :param pin.Model robot: Robot model data, used to get joint limits.
    :return: List of servo inputs
    """
    servo_commands = []
    for q in path_plan:
        perc_travel = 2.0 * ((q - robot.model.lowerPositionLimit) / (
                    robot.model.upperPositionLimit - robot.model.lowerPositionLimit)) - 1.0
        servo_commands.append(perc_travel * 500 + 500)
    return servo_commands


def solve_ik_with_joint_limits(robot, q_start, next_position, gamma=0.1, tol=1E-5, max_iters=5E3):
    """
    Utilises the Levenberg-Marquardt [damped least-squares] algorithm to minimize the pose error from the desired pose
    and the robots 'wire' frame pose.

    Derived from: https://github.com/tbretl/ae598-arp/blob/main/examples/20260210_IK_as_QP_template.ipynb

    :param robot: Pinocchio robot model.
    :param np.ndarray q_start: Starting configuration of the robot.
    :param np.ndarray next_position: Desired pose for the next movement.
    :param float gamma: Dampening factor.
    :param float tol: Desired tolerance in error before breaking out of interactions early.
    :param int max_iters: Maximum number of iterations to calculate.
    :return: q_next: required joint positions to reach desired pose.
    """
    # Initial configuration
    q = q_start.copy()

    # Current number of iterations
    curr_iters = 0

    while True:
        pin.forwardKinematics(robot.model, robot.data, q)
        pin.updateFramePlacements(robot.model, robot.data)

        A = robot.data.oMf[robot.model.getFrameId('wire')].homogeneous
        pin.computeJointJacobians(robot.model, robot.data)

        J_b = pin.getFrameJacobian(robot.model, robot.data, robot.model.getFrameId('wire'), pin.LOCAL)
        v_b_error = np.array(pin.log6(pin.SE3(np.linalg.inv(A) @ next_position)))

        Jlog6 = pin.Jlog6(pin.SE3(np.linalg.inv(next_position) @ A))
        r, J = v_b_error, -Jlog6 @ J_b

        # Complete early if the tolerance is satisfied
        if 0.5 * np.dot(r, r) < tol:
            break
        # Check if maximum number of iterations has been reached
        if curr_iters > max_iters:
            print(f'FAILURE (exceeded {max_iters} iterations)')
            break

        # Do one step of LM
        dq = solve_qp(
            J.T @ J + gamma**2 * np.identity(len(q)),
            J.T @ r,
            lb=robot.model.lowerPositionLimit - q,
            ub=robot.model.upperPositionLimit - q,
            solver='proxqp',
        )
        q += dq

        # Check if stopping criterion ON PROGRESS is met
        if np.allclose(dq, 0):
            print(f'Failed to reach desired pose, but converged, desired cut path may not be satisfied.')
            break

        # Increment number of iterations
        curr_iters += 1

        # Update display
        pin.forwardKinematics(robot.model, robot.data, q)
        pin.updateFramePlacements(robot.model, robot.data)

    return q


def point_on_path_to_pose(wire_mid_point, wire_direction_vector, wire_forward_vector):
    pose = np.zeros([4, 4])
    pose[:3, 3] = wire_mid_point
    pose[3, 3] = 1

    wire_up_vector = np.cross(wire_forward_vector, wire_direction_vector)
    pose[:3, 0] = wire_forward_vector
    pose[:3, 1] = wire_direction_vector
    pose[:3, 2] = wire_up_vector

    return pose


def path_plan_static_plate(q_start, curr_section, reference_voxel, thermal_voxel, robot, reverse_path):
    """
    Generate a motion path of robot joint inputs to move anywhere in the workspace with the cost function of maximizing
    the removal of material from thermal_voxel with respect ot reference_voxel.

    :param q_start:
    :param curr_section:
    :param reference_voxel:
    :param thermal_voxel:
    :param robot:
    :return:
    """
    joint_angles = list()
    path = curr_section.get_path()
    if reverse_path:
        path.reverse()
    for point in path:
        wire_mid_point = np.array([point[0], point[1], point[2]]) / 1000  # Convert mm to m
        angle = min((wire_mid_point[2] / 0.12) * np.pi / 10 - np.pi / 10.0, np.pi / 10.0)
        wire_direction_vector = np.array([0.0, 1.0, 0.0])
        wire_forward_vector = np.array([np.sin(angle), 0.0, -np.cos(angle)])
        next_position = point_on_path_to_pose(wire_mid_point, wire_direction_vector, wire_forward_vector)
        q = solve_ik_with_joint_limits(robot, q_start, next_position, gamma=0.1)
        joint_angles.append(q)
    return np.array(joint_angles)


def add_base_and_top_cut(cross_sections, top_height):
    new_sections = list()
    for section in cross_sections:
        section_path = copy.deepcopy(section.section_list[0]).get_path()
        path_start = copy.deepcopy(section_path[0])
        base_path = list()
        for i in range(32):
            new_point = copy.deepcopy(path_start)
            new_point[0] -= (32 - i) / 32.0 * 30
            base_path.append(copy.deepcopy(new_point))
        base_path.extend(section_path)
        section_path = copy.deepcopy(base_path)
        path_end = copy.deepcopy(section_path[-1])
        for i in range(1, 33):
            new_point = copy.deepcopy(path_end)
            new_point[2] += (i) / 32.0 * (top_height * 1000)
            section_path.append(copy.deepcopy(new_point))
        new_sections.append(CrossSection([primPath(section_path)]))
    return new_sections


def compensate_for_melt_front(cross_sections, workpiece_material, wire_cutter_desc):
    new_cross_sections = list()
    electrical_resistance = wire_cutter_desc.rho * wire_cutter_desc.length
    electrical_power = wire_cutter_desc.current ** 2 / electrical_resistance
    melt_temp = workpiece_material.melt_temp
    wire_temp = lookup_nichrome_temp(wire_cutter_desc.current)
    heat_conductance = workpiece_material.heat_conductance

    melt_radius = 0.001
    area = 2 * np.pi * (wire_cutter_desc.radius + melt_radius) * wire_cutter_desc.length
    while np.abs(area * heat_conductance / electrical_power * (wire_temp - melt_temp) - melt_radius) > 1E-5:
        area = 2 * np.pi * (wire_cutter_desc.radius + melt_radius) * wire_cutter_desc.length
        melt_radius = area * heat_conductance / electrical_power * (wire_temp - melt_temp)

    # Convert from meters to millimeters
    melt_radius *= 1000.0
    for section in cross_sections:
        new_points = list()
        base_path = section.get_path()
        for i in range(len(base_path) - 1):
            r = base_path[i + 1] - base_path[i]
            r = np.array([r[0], r[1], r[2]])
            normal = np.cross(r, np.array([0.0, 1.0, 0.0]))
            new_points.append(Point(base_path[i][0] + melt_radius * normal[0], base_path[i][1],
                                    base_path[i][2] + melt_radius * normal[2]))
        new_cross_sections.append(CrossSection([primPath(new_points)]))
    return new_cross_sections


def lookup_nichrome_temp(current):
    """
    Look up function for wire temp at a given current for 25 AWG nicrhome wire.
    :param current:
    """
    amp = np.array([1.92, 2.52, 3.0, 3.6, 4.3, 5.2])
    temp = np.array([205, 315, 427, 538, 649, 760])
    poly = np.polynomial.polynomial.Polynomial.fit(amp, temp, deg=4)

    return poly(current)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='RobotDemo',
        description='Generates g-code for a 6dof robot with a fixed thermal end-effector to cut foam. Requires an '
                    'input geometry [stl] along with the robot description [pinnochio input file]. '
                    'Dependencies: \n-Trimesh\n-Pinnochio\n-crocodyl')
    parser.add_argument('-stl', type=str, help='Filepath to the desired stl file to be cut')
    parser.add_argument('-robot', type=str, help='Filepath to the pinnochio compatible robot description')
    parser.add_argument('-workpiece_material', type=str, help='Filepath to the material description file')
    parser.add_argument('-wire_cutter_desc', type=str, help='Filepath to the file describing wire cutter properties')
    parser.add_argument('-out_dir', type=str, help='Filepath to the directory to save generated gcode in')
    parser.add_argument('-cell_resolution', type=float, default=1.0, help='Edge length of a single voxel cell in mm')
    parser.add_argument('-theta_resolution', type=float, default=45.0,
                        help='Rotation resolution for cross sections in deg')
    parser.add_argument('-tolerance', type=float, default=1E-2, help='Tolerance to cut to')
    parser.add_argument('--debug', action='store_true',
                        help='If enabled will create a debug directory in -out_dir and save intermediate steps')

    args = vars(parser.parse_args())

    stl_file_path = args['stl']
    robot_file_path = args['robot']
    workpiece_material = args['workpiece_material']
    wire_cutter_desc = args['wire_cutter_desc']
    output_dir = args['out_dir']
    cell_resolution = args['cell_resolution']
    theta_resolution = args['theta_resolution']
    robot_filepath = args['robot']
    tolerance = args['tolerance']
    debug = args['debug']

    main(stl_file_path, robot_file_path, workpiece_material, wire_cutter_desc, output_dir, cell_resolution,
         theta_resolution, tolerance, debug)
