from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  return LaunchDescription(OpaqueFunction(function=launch_setup))


def launch_setup(context):

  nav_params_file = PathJoinSubstitution([FindPackageShare('pdor_bringup'), 'config', 'nav_params.yaml'])
  
  bt_navigator_node = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    output='screen',
    parameters=[nav_params_file]
  )

  map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    parameters=[nav_params_file]
  )

  amcl_node = Node(
    package="nav2_amcl",
    executable="amcl",
    output="screen",
    parameters=[nav_params_file]
  )

  planner_server_node = Node(
    package="nav2_planner",
    executable="planner_server",
    output="screen",
    parameters=[nav_params_file]
  )

  controller_server_node = Node(
    package='nav2_controller',
    executable='controller_server',
    parameters=[nav_params_file]
  )

  lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    output='screen',
    parameters=[{'node_names' : ['controller_server',
                                 'planner_server',
                                 'bt_navigator',
                                 'map_server',
                                 'amcl',
                                 'planner_server',
                                 'controller_server'],
                  'bond_timeout' : 0.0,
                  'autostart' : True}]
  )

  return [
    lifecycle_manager_node,
    bt_navigator_node,
    map_server_node,
    amcl_node,
    planner_server_node,
    controller_server_node
  ]