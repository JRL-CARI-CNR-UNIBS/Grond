from launch_ros.actions.push_ros_namespace import PushRosNamespace
from launch.launch_description import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction, GroupAction

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import Parameter

import xacro

def generate_launch_description():
  args = [

  ]

  return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])

def launch_setup(context):

  robot_description_path = PathJoinSubstitution([FindPackageShare('grond_description'), 'urdf','system.urdf.xacro']).perform(context)
  robot_description_args = {
    'name' : 'grond',
  }
  robot_description = xacro.process(input_file_name=robot_description_path, mappings=robot_description_args).toprettyxml(indent=' ')

  ros2_controllers_config = PathJoinSubstitution([FindPackageShare('grond_bringup'), 'config', 'imm.ros2_control.yaml'])

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[Parameter(name='robot_description', value=robot_description, value_type=str)]
  )

  controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[ros2_controllers_config],
    remappings=[
      ('controller_manager/robot_description', 'robot_description'),
      ('mecanum_controller/tf_odometry', '/tf'),
      ('mecanum_controller/cmd_vel_unstamped','cmd_vel')
    ]
  )

  joint_state_publisher_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_publisher',
      '--controller-manager', 'controller_manager']
  )

  mecanum_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['mecanum_controller']
  )

  admittance_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_publisher',
      '--controller-manager', 'controller_manager']
  )

  return GroupAction([
    PushRosNamespace('grond'),
    robot_state_publisher,
    controller_manager,
    joint_state_publisher_spawner,
    admittance_controller_spawner,
    #mecanum_controller_spawner
  ])
