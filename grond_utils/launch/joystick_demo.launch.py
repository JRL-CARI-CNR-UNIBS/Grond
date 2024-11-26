from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = 'controllers.yaml'

    description_package = 'grond_description'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'platform.urdf.xacro']),
            
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, initial_joint_controllers],
        output='screen',
    )

#    velocity_controller_node = Node(
#        package='controller_manager',
#        executable='spawner',
#        arguments=['forward_velocity_controller'],
#        output='screen',
#    )

    velocity_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller'],
        output='screen',
    )


    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    joystick_publisher = Node(
        package='grond_utils',
        executable='joystick_publisher',
        output='screen',
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # declared_arguments

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(velocity_controller_node)
    ld.add_action(jsb)
    ld.add_action(joy_node)
    ld.add_action(joystick_publisher)

    return ld