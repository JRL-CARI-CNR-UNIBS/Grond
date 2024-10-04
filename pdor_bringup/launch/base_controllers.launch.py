from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription(OpaqueFunction(function=launch_setup))


def launch_setup(context):

  controller_params_file = PathJoinSubstitution([FindPackageShare('pdor_bringup'), 'config', 'base.ros2_control.yaml'])

  controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    output='screen',
    parameters=[controller_params_file],
    remappings=[('/controller_manager/robot_description', '/robot_description')]
  )

  mecanum_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    parameters=['mecanum_controller']
  )

  joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
  )

  return [
    controller_manager_node,
    mecanum_controller_spawner,
    joint_state_broadcaster_node
  ]