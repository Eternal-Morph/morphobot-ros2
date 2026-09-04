import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('stm32_drone_control')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Launch Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 Visualization'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='STM32 Serial Port Path'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='STM32 Serial Port Baud Rate'
    )

    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge',
        default_value='true',
        description='Launch STM32 HIL Bridge Node'
    )

    # Gazebo 3D Mesh Resource Paths
    pkg_parent_share = os.path.dirname(pkg_share)
    ign_resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        pkg_parent_share + (':' + os.environ['IGN_GAZEBO_RESOURCE_PATH'] if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ else '')
    )
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        pkg_parent_share + (':' + os.environ['GZ_SIM_RESOURCE_PATH'] if 'GZ_SIM_RESOURCE_PATH' in os.environ else '')
    )

    # URDF Path and Robot Description
    urdf_path = os.path.join(pkg_share, 'urdf', 'drone.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Robot State Publisher Node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo Harmonic Simulation Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn Drone into Gazebo
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'stm32_drone',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ROS 2 <-> Gazebo Harmonic Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            '/stm32_drone/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
            '/imu/sim_data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # STM32 HIL Bridge Node
    hil_bridge_node = Node(
        package='stm32_drone_control',
        executable='stm32_hil_bridge',
        name='stm32_hil_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('port'),
            'baud_rate': LaunchConfiguration('baudrate'),
            'imu_topic': '/imu/sim_data',
            'motor_topic': '/stm32_drone/command/motor_speed'
        }],
        condition=IfCondition(LaunchConfiguration('use_bridge'))
    )

    # RViz2 Visualization Node
    rviz_config_path = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        port_arg,
        baudrate_arg,
        use_bridge_arg,
        ign_resource_env,
        gz_resource_env,
        rsp_node,
        gazebo,
        spawn_drone,
        bridge,
        hil_bridge_node,
        rviz_node
    ])
