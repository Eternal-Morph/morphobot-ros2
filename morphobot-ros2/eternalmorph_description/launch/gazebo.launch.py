import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('eternalmorph_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    pkg_parent_dir = os.path.dirname(pkg_share)
    colcon_gz_plugin_dir = '/home/emirhan/colcon_ws/install/gz_ros2_control/lib'

    # Ensure Gazebo Harmonic plugin path includes the harmonic-built gz_ros2_control
    if colcon_gz_plugin_dir not in os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''):
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = colcon_gz_plugin_dir + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if colcon_gz_plugin_dir not in os.environ.get('LD_LIBRARY_PATH', ''):
        os.environ['LD_LIBRARY_PATH'] = colcon_gz_plugin_dir + ':' + os.environ.get('LD_LIBRARY_PATH', '')

    default_model_path = os.path.join(pkg_share, 'urdf', 'eternalmorph.xacro')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot URDF/xacro file'
    )

    # Environment variables for Gazebo 3D Mesh resolution & gz_ros2_control
    ign_resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        pkg_parent_dir + (':' + os.environ['IGN_GAZEBO_RESOURCE_PATH'] if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ else '')
    )
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        pkg_parent_dir + (':' + os.environ['GZ_SIM_RESOURCE_PATH'] if 'GZ_SIM_RESOURCE_PATH' in os.environ else '')
    )
    gz_plugin_env = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH']
    )
    ld_lib_env = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        os.environ['LD_LIBRARY_PATH']
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

    # Gazebo Harmonic Simulation Launch (gz_sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
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

    # ROS 2 Control Spawners (rover_ws ile birebir mimari)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            os.path.join(pkg_share, 'config', 'controllers.yaml')
        ],
        output='screen'
    )

    # Robot olustuktan sonra controller'lari sirayla baslat
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # ROS-GZ Bridge arguments for Gazebo Harmonic
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
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

    return LaunchDescription([
        ign_resource_env,
        gz_resource_env,
        gz_plugin_env,
        ld_lib_env,
        model_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        bridge
    ])

