import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('stm32_drone_control')

    # Launch Argümanları
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='STM32 UART Seri Port Yolu'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='STM32 UART Baud Rate'
    )

    invert_roll_arg = DeclareLaunchArgument(
        'invert_roll',
        default_value='False',
        description='Roll eksenini ters çevir'
    )
    invert_pitch_arg = DeclareLaunchArgument(
        'invert_pitch',
        default_value='False',
        description='Pitch eksenini ters çevir'
    )
    invert_yaw_arg = DeclareLaunchArgument(
        'invert_yaw',
        default_value='False',
        description='Yaw eksenini ters çevir'
    )

    # Dosya Yolları
    urdf_path = os.path.join(pkg_share, 'urdf', 'drone.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    pid_params_path = os.path.join(pkg_share, 'config', 'pid_params.yaml')

    # Robot State Publisher (ParameterValue sarmalayıcısı)
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Seri Port IMU Bridge Düğümü
    serial_bridge_node = Node(
        package='stm32_drone_control',
        executable='serial_imu_bridge',
        name='serial_imu_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'invert_roll': LaunchConfiguration('invert_roll'),
            'invert_pitch': LaunchConfiguration('invert_pitch'),
            'invert_yaw': LaunchConfiguration('invert_yaw')
        }]
    )

    # PID Tuner Düğümü
    pid_tuner_node = Node(
        package='stm32_drone_control',
        executable='pid_tuner_node',
        name='pid_tuner_node',
        output='screen',
        parameters=[pid_params_path]
    )

    # RViz2 Düğümü
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        invert_roll_arg,
        invert_pitch_arg,
        invert_yaw_arg,
        rsp_node,
        serial_bridge_node,
        pid_tuner_node,
        rviz_node
    ])
