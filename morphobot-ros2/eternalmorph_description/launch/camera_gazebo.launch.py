import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('eternalmorph_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    pkg_parent_dir = os.path.dirname(pkg_share)

    default_model_path = os.path.join(pkg_share, 'urdf', 'eternalmorph.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    default_rviz_config_path = os.path.join(pkg_share, 'launch', 'camera.rviz')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF/xacro file'
    )

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Absolute path to world SDF file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Flag to enable RViz2'
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )

    # Environment variables for Gazebo 3D Mesh resolution
    ign_resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        pkg_parent_dir + (':' + os.environ['IGN_GAZEBO_RESOURCE_PATH'] if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ else '')
    )
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        pkg_parent_dir + (':' + os.environ['GZ_SIM_RESOURCE_PATH'] if 'GZ_SIM_RESOURCE_PATH' in os.environ else '')
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Gazebo Harmonic Simulation Launch (gz_sim) with sensor-enabled world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
    )

    # Spawn robot in Gazebo Harmonic
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

    # ROS-GZ Bridge arguments for Gazebo Harmonic (including camera topics)
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/world/empty/model/eternalmorph/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/eternalmorph/sol_on_ground_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/sol_on_air_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/sag_on_ground_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/sag_on_air_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/sol_arka_ground_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/sol_arka_air_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/sag_arka_ground_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/sag_arka_air_mode/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/eternalmorph/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/imu/sim_data@sensor_msgs/msg/Imu[gz.msgs.IMU',
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=[
            ('/world/empty/model/eternalmorph/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        ign_resource_env,
        gz_resource_env,
        model_arg,
        world_arg,
        rviz_arg,
        rvizconfig_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge,
        rviz_node
    ])
