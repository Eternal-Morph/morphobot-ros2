import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('eternalmorph_description')
    pkg_parent_dir = os.path.abspath(os.path.join(pkg_share, '..'))

    default_model_path = os.path.join(pkg_share, 'urdf', 'eternalmorph.xacro')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF/xacro file'
    )

    # Set GZ_SIM_RESOURCE_PATH so Gazebo Sim resolves package://eternalmorph_description/
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_parent_dir
    )

    robot_description = Command([
        'xacro ', LaunchConfiguration('model')
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Gazebo Sim Launch (gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'eternalmorph',
            '-z', '0.3'
        ],
        output='screen'
    )

    # ROS-GZ Bridge arguments
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/eternalmorph/sol_on_ground_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        '/eternalmorph/sol_on_air_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        '/eternalmorph/sag_on_ground_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        '/eternalmorph/sag_on_air_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        '/eternalmorph/sol_arka_ground_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        '/eternalmorph/sol_arka_air_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        '/eternalmorph/sag_arka_ground_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
        '/eternalmorph/sag_arka_air_mode/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        model_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge
    ])
