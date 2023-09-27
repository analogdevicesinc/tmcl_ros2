# Copyright (c) 2023 Analog Devices, Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (LaunchConfiguration, PythonExpression)
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  ## Get directory of config files
  tmcl_ros2_pkg_dir = get_package_share_directory('tmcl_ros2')
  ## Include YAML file
  autogenerated_yaml = os.path.join(tmcl_ros2_pkg_dir, 'config', 'autogenerated', 'TMCM-6210.yaml')
  ext_yaml = os.path.join(tmcl_ros2_pkg_dir, 'config', 'TMCM-6210_Ext.yaml')

  # Declare the Launch Arguments with default value
  arg_log_level = DeclareLaunchArgument('log_level', default_value='INFO')
  log_level_ = LaunchConfiguration('log_level')

  arg_ns_prefix = DeclareLaunchArgument('ns_prefix', default_value='tmcm1')
  ns_prefix_ = LaunchConfiguration('ns_prefix')

  arg_tmcl_base_name = DeclareLaunchArgument('tmcl_base_name', default_value='tmcm6210')
  tmcl_base_name_ = LaunchConfiguration('tmcl_base_name')

  arg_board_parent_frame = DeclareLaunchArgument('board_parent_frame', default_value='map')
  arg_board_base_name = DeclareLaunchArgument('board_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_base_link'])
  arg_board_pos_x = DeclareLaunchArgument('board_pos_x', default_value='0')
  arg_board_pos_y = DeclareLaunchArgument('board_pos_y', default_value='0')
  arg_board_pos_z = DeclareLaunchArgument('board_pos_z', default_value='0')
  arg_board_pos_qx = DeclareLaunchArgument('board_pos_qx', default_value='0')
  arg_board_pos_qy = DeclareLaunchArgument('board_pos_qy', default_value='0')
  arg_board_pos_qz = DeclareLaunchArgument('board_pos_qz', default_value='0')
  arg_board_pos_qw = DeclareLaunchArgument('board_pos_qw', default_value='1')

  board_parent_frame_ = LaunchConfiguration('board_parent_frame')
  board_base_name_ = LaunchConfiguration('board_base_name')
  board_pos_x_ = LaunchConfiguration('board_pos_x')
  board_pos_y_ = LaunchConfiguration('board_pos_y')
  board_pos_z_ = LaunchConfiguration('board_pos_z')
  board_pos_qx_ = LaunchConfiguration('board_pos_qx')
  board_pos_qy_ = LaunchConfiguration('board_pos_qy')
  board_pos_qz_ = LaunchConfiguration('board_pos_qz')
  board_pos_qw_ = LaunchConfiguration('board_pos_qw')
  # motor0 Arguments
  arg_mtr0_base_name = DeclareLaunchArgument('mtr0_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_mtr0_frame'])
  arg_mtr0_pos_x = DeclareLaunchArgument('mtr0_pos_x', default_value='1')
  arg_mtr0_pos_y = DeclareLaunchArgument('mtr0_pos_y', default_value='0')
  arg_mtr0_pos_z = DeclareLaunchArgument('mtr0_pos_z', default_value='0')
  arg_mtr0_pos_qx = DeclareLaunchArgument('mtr0_pos_qx', default_value='0')
  arg_mtr0_pos_qy = DeclareLaunchArgument('mtr0_pos_qy', default_value='0')
  arg_mtr0_pos_qz = DeclareLaunchArgument('mtr0_pos_qz', default_value='0')
  arg_mtr0_pos_qw = DeclareLaunchArgument('mtr0_pos_qw', default_value='1')

  mtr0_base_name_ = LaunchConfiguration('mtr0_base_name')
  mtr0_pos_x_ = LaunchConfiguration('mtr0_pos_x')
  mtr0_pos_y_ = LaunchConfiguration('mtr0_pos_y')
  mtr0_pos_z_ = LaunchConfiguration('mtr0_pos_z')
  mtr0_pos_qx_ = LaunchConfiguration('mtr0_pos_qx')
  mtr0_pos_qy_ = LaunchConfiguration('mtr0_pos_qy')
  mtr0_pos_qz_ = LaunchConfiguration('mtr0_pos_qz')
  mtr0_pos_qw_ = LaunchConfiguration('mtr0_pos_qw')

  # motor1 Arguments
  arg_mtr1_base_name = DeclareLaunchArgument('mtr1_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_mtr0_frame'])
  arg_mtr1_pos_x = DeclareLaunchArgument('mtr1_pos_x', default_value='1')
  arg_mtr1_pos_y = DeclareLaunchArgument('mtr1_pos_y', default_value='0')
  arg_mtr1_pos_z = DeclareLaunchArgument('mtr1_pos_z', default_value='0')
  arg_mtr1_pos_qx = DeclareLaunchArgument('mtr1_pos_qx', default_value='0')
  arg_mtr1_pos_qy = DeclareLaunchArgument('mtr1_pos_qy', default_value='0')
  arg_mtr1_pos_qz = DeclareLaunchArgument('mtr1_pos_qz', default_value='0')
  arg_mtr1_pos_qw = DeclareLaunchArgument('mtr1_pos_qw', default_value='1')

  mtr1_base_name_ = LaunchConfiguration('mtr1_base_name')
  mtr1_pos_x_ = LaunchConfiguration('mtr1_pos_x')
  mtr1_pos_y_ = LaunchConfiguration('mtr1_pos_y')
  mtr1_pos_z_ = LaunchConfiguration('mtr1_pos_z')
  mtr1_pos_qx_ = LaunchConfiguration('mtr1_pos_qx')
  mtr1_pos_qy_ = LaunchConfiguration('mtr1_pos_qy')
  mtr1_pos_qz_ = LaunchConfiguration('mtr1_pos_qz')
  mtr1_pos_qw_ = LaunchConfiguration('mtr1_pos_qw')

  # motor2 Arguments
  arg_mtr2_base_name = DeclareLaunchArgument('mtr2_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_mtr0_frame'])
  arg_mtr2_pos_x = DeclareLaunchArgument('mtr2_pos_x', default_value='1')
  arg_mtr2_pos_y = DeclareLaunchArgument('mtr2_pos_y', default_value='0')
  arg_mtr2_pos_z = DeclareLaunchArgument('mtr2_pos_z', default_value='0')
  arg_mtr2_pos_qx = DeclareLaunchArgument('mtr2_pos_qx', default_value='0')
  arg_mtr2_pos_qy = DeclareLaunchArgument('mtr2_pos_qy', default_value='0')
  arg_mtr2_pos_qz = DeclareLaunchArgument('mtr2_pos_qz', default_value='0')
  arg_mtr2_pos_qw = DeclareLaunchArgument('mtr2_pos_qw', default_value='1')

  mtr2_base_name_ = LaunchConfiguration('mtr2_base_name')
  mtr2_pos_x_ = LaunchConfiguration('mtr2_pos_x')
  mtr2_pos_y_ = LaunchConfiguration('mtr2_pos_y')
  mtr2_pos_z_ = LaunchConfiguration('mtr2_pos_z')
  mtr2_pos_qx_ = LaunchConfiguration('mtr2_pos_qx')
  mtr2_pos_qy_ = LaunchConfiguration('mtr2_pos_qy')
  mtr2_pos_qz_ = LaunchConfiguration('mtr2_pos_qz')
  mtr2_pos_qw_ = LaunchConfiguration('mtr2_pos_qw')

  # motor3 Arguments
  arg_mtr3_base_name = DeclareLaunchArgument('mtr3_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_mtr0_frame'])
  arg_mtr3_pos_x = DeclareLaunchArgument('mtr3_pos_x', default_value='1')
  arg_mtr3_pos_y = DeclareLaunchArgument('mtr3_pos_y', default_value='0')
  arg_mtr3_pos_z = DeclareLaunchArgument('mtr3_pos_z', default_value='0')
  arg_mtr3_pos_qx = DeclareLaunchArgument('mtr3_pos_qx', default_value='0')
  arg_mtr3_pos_qy = DeclareLaunchArgument('mtr3_pos_qy', default_value='0')
  arg_mtr3_pos_qz = DeclareLaunchArgument('mtr3_pos_qz', default_value='0')
  arg_mtr3_pos_qw = DeclareLaunchArgument('mtr3_pos_qw', default_value='1')

  mtr3_base_name_ = LaunchConfiguration('mtr3_base_name')
  mtr3_pos_x_ = LaunchConfiguration('mtr3_pos_x')
  mtr3_pos_y_ = LaunchConfiguration('mtr3_pos_y')
  mtr3_pos_z_ = LaunchConfiguration('mtr3_pos_z')
  mtr3_pos_qx_ = LaunchConfiguration('mtr3_pos_qx')
  mtr3_pos_qy_ = LaunchConfiguration('mtr3_pos_qy')
  mtr3_pos_qz_ = LaunchConfiguration('mtr3_pos_qz')
  mtr3_pos_qw_ = LaunchConfiguration('mtr3_pos_qw')

  # motor4 Arguments
  arg_mtr4_base_name = DeclareLaunchArgument('mtr4_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_mtr0_frame'])
  arg_mtr4_pos_x = DeclareLaunchArgument('mtr4_pos_x', default_value='1')
  arg_mtr4_pos_y = DeclareLaunchArgument('mtr4_pos_y', default_value='0')
  arg_mtr4_pos_z = DeclareLaunchArgument('mtr4_pos_z', default_value='0')
  arg_mtr4_pos_qx = DeclareLaunchArgument('mtr4_pos_qx', default_value='0')
  arg_mtr4_pos_qy = DeclareLaunchArgument('mtr4_pos_qy', default_value='0')
  arg_mtr4_pos_qz = DeclareLaunchArgument('mtr4_pos_qz', default_value='0')
  arg_mtr4_pos_qw = DeclareLaunchArgument('mtr4_pos_qw', default_value='1')

  mtr4_base_name_ = LaunchConfiguration('mtr4_base_name')
  mtr4_pos_x_ = LaunchConfiguration('mtr4_pos_x')
  mtr4_pos_y_ = LaunchConfiguration('mtr4_pos_y')
  mtr4_pos_z_ = LaunchConfiguration('mtr4_pos_z')
  mtr4_pos_qx_ = LaunchConfiguration('mtr4_pos_qx')
  mtr4_pos_qy_ = LaunchConfiguration('mtr4_pos_qy')
  mtr4_pos_qz_ = LaunchConfiguration('mtr4_pos_qz')
  mtr4_pos_qw_ = LaunchConfiguration('mtr4_pos_qw')

  # motor5 Arguments
  arg_mtr5_base_name = DeclareLaunchArgument('mtr5_base_name', default_value=[ns_prefix_,'/',tmcl_base_name_,'_mtr0_frame'])
  arg_mtr5_pos_x = DeclareLaunchArgument('mtr5_pos_x', default_value='1')
  arg_mtr5_pos_y = DeclareLaunchArgument('mtr5_pos_y', default_value='0')
  arg_mtr5_pos_z = DeclareLaunchArgument('mtr5_pos_z', default_value='0')
  arg_mtr5_pos_qx = DeclareLaunchArgument('mtr5_pos_qx', default_value='0')
  arg_mtr5_pos_qy = DeclareLaunchArgument('mtr5_pos_qy', default_value='0')
  arg_mtr5_pos_qz = DeclareLaunchArgument('mtr5_pos_qz', default_value='0')
  arg_mtr5_pos_qw = DeclareLaunchArgument('mtr5_pos_qw', default_value='1')

  mtr5_base_name_ = LaunchConfiguration('mtr5_base_name')
  mtr5_pos_x_ = LaunchConfiguration('mtr5_pos_x')
  mtr5_pos_y_ = LaunchConfiguration('mtr5_pos_y')
  mtr5_pos_z_ = LaunchConfiguration('mtr5_pos_z')
  mtr5_pos_qx_ = LaunchConfiguration('mtr5_pos_qx')
  mtr5_pos_qy_ = LaunchConfiguration('mtr5_pos_qy')
  mtr5_pos_qz_ = LaunchConfiguration('mtr5_pos_qz')
  mtr5_pos_qw_ = LaunchConfiguration('mtr5_pos_qw')


  # TMCL ROS2 Node
  tmcm_6210_node = Node(
    package="tmcl_ros2",
    executable="tmcl_ros2_node",
    name="tmcl_ros2_node",
    namespace=ns_prefix_,
    emulate_tty=True,
    parameters=[
      autogenerated_yaml,
      ext_yaml],
    arguments=["--ros-args", "--log-level", PythonExpression(expression=["'",ns_prefix_,".tmcl_ros2_node:=",log_level_,"'"])]
  )

  # TF Node
  tf_board = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", board_pos_x_, "--y", board_pos_y_, "--z", board_pos_z_,
      "--qx", board_pos_qx_, "--qy", board_pos_qy_, "--qz", board_pos_qz_, "--qw", board_pos_qw_,
      "--frame-id", board_parent_frame_, "--child-frame-id", board_base_name_]
  )

  tf_mtr0 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_mtr0_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", mtr0_pos_x_, "--y", mtr0_pos_y_, "--z", mtr0_pos_z_,
      "--qx", mtr0_pos_qx_, "--qy", mtr0_pos_qy_, "--qz", mtr0_pos_qz_, "--qw", mtr0_pos_qw_,
      "--frame-id", board_base_name_, "--child-frame-id", mtr0_base_name_]
  )

  tf_mtr1 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_mtr1_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", mtr1_pos_x_, "--y", mtr1_pos_y_, "--z", mtr1_pos_z_,
      "--qx", mtr1_pos_qx_, "--qy", mtr1_pos_qy_, "--qz", mtr1_pos_qz_, "--qw", mtr1_pos_qw_,
      "--frame-id", board_base_name_, "--child-frame-id", mtr1_base_name_]
  )

  tf_mtr2 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_mtr2_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", mtr2_pos_x_, "--y", mtr2_pos_y_, "--z", mtr2_pos_z_,
      "--qx", mtr2_pos_qx_, "--qy", mtr2_pos_qy_, "--qz", mtr2_pos_qz_, "--qw", mtr2_pos_qw_,
      "--frame-id", board_base_name_, "--child-frame-id", mtr2_base_name_]
  )

  tf_mtr3 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_mtr3_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", mtr3_pos_x_, "--y", mtr3_pos_y_, "--z", mtr3_pos_z_,
      "--qx", mtr3_pos_qx_, "--qy", mtr3_pos_qy_, "--qz", mtr3_pos_qz_, "--qw", mtr3_pos_qw_,
      "--frame-id", board_base_name_, "--child-frame-id", mtr3_base_name_]
  )

  tf_mtr4 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_mtr4_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", mtr4_pos_x_, "--y", mtr4_pos_y_, "--z", mtr4_pos_z_,
      "--qx", mtr4_pos_qx_, "--qy", mtr4_pos_qy_, "--qz", mtr4_pos_qz_, "--qw", mtr4_pos_qw_,
      "--frame-id", board_base_name_, "--child-frame-id", mtr4_base_name_]
  )

  tf_mtr5 = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name=[tmcl_base_name_,'_mtr5_link'],
    namespace=ns_prefix_,
    emulate_tty=True,
    arguments=["--x", mtr5_pos_x_, "--y", mtr5_pos_y_, "--z", mtr5_pos_z_,
      "--qx", mtr5_pos_qx_, "--qy", mtr5_pos_qy_, "--qz", mtr5_pos_qz_, "--qw", mtr5_pos_qw_,
      "--frame-id", board_base_name_, "--child-frame-id", mtr5_base_name_]
  )

  return LaunchDescription([
      arg_log_level,
      arg_ns_prefix,
      arg_tmcl_base_name,
      arg_board_parent_frame,
      arg_board_base_name,arg_board_pos_x, arg_board_pos_y, arg_board_pos_z, arg_board_pos_qx, arg_board_pos_qy, arg_board_pos_qz, arg_board_pos_qw,
      arg_mtr0_base_name, arg_mtr0_pos_x, arg_mtr0_pos_y, arg_mtr0_pos_z, arg_mtr0_pos_qx, arg_mtr0_pos_qy, arg_mtr0_pos_qz, arg_mtr0_pos_qw,
      arg_mtr1_base_name, arg_mtr1_pos_x, arg_mtr1_pos_y, arg_mtr1_pos_z, arg_mtr1_pos_qx, arg_mtr1_pos_qy, arg_mtr1_pos_qz, arg_mtr1_pos_qw,
      arg_mtr2_base_name, arg_mtr2_pos_x, arg_mtr2_pos_y, arg_mtr2_pos_z, arg_mtr2_pos_qx, arg_mtr2_pos_qy, arg_mtr2_pos_qz, arg_mtr2_pos_qw,
      arg_mtr3_base_name, arg_mtr3_pos_x, arg_mtr3_pos_y, arg_mtr3_pos_z, arg_mtr3_pos_qx, arg_mtr3_pos_qy, arg_mtr3_pos_qz, arg_mtr3_pos_qw,
      arg_mtr4_base_name, arg_mtr4_pos_x, arg_mtr4_pos_y, arg_mtr4_pos_z, arg_mtr4_pos_qx, arg_mtr4_pos_qy, arg_mtr4_pos_qz, arg_mtr4_pos_qw,
      arg_mtr5_base_name, arg_mtr5_pos_x, arg_mtr5_pos_y, arg_mtr5_pos_z, arg_mtr5_pos_qx, arg_mtr5_pos_qy, arg_mtr5_pos_qz, arg_mtr5_pos_qw,
      tmcm_6210_node,
      tf_board,
      tf_mtr0,
      tf_mtr1,
      tf_mtr2,
      tf_mtr3,
      tf_mtr4,
      tf_mtr5,
  ])
