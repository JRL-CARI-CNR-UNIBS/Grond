from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, FindExecutable, LaunchConfiguration, Command
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  args = list()
  args.append(
    DeclareLaunchArgument(name='pub_robot_description', default_value='false', description='create robot_state_publisher to publish robot_descritpion')
  )

  return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):

  controller_params_file = PathJoinSubstitution([FindPackageShare('pdor_bringup'), 'config', 'base.ros2_control.yaml'])

  # Used only if pub_robot_description is true
  robot_description = Command(
      command=[FindExecutable(name='xacro'), ' ',
           PathJoinSubstitution([FindPackageShare('pdor_description'), 'urdf', 'platform.urdf.xacro'])
      ]
    )

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    condition=IfCondition(LaunchConfiguration('pub_robot_description')),
    parameters=[{'robot_description' : ParameterValue(value=robot_description, value_type=str)}]
  )

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
    arguments=['mecanum_controller']
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
    joint_state_broadcaster_node,
    robot_state_publisher_node
  ]