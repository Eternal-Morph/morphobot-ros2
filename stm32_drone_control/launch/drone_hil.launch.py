import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    # Dosya Yolları
    rviz_config_path = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    pid_params_path = os.path.join(pkg_share, 'config', 'pid_params.yaml')

    # 1. Gerçek Seri Port IMU Bridge Düğümü (Donanımı dinler ve RViz2 için TF yayınlar)
    serial_bridge_node = Node(
        package='stm32_drone_control',
        executable='serial_imu_bridge',
        name='serial_imu_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )

    # 2. HIL Real-to-Sim Köprüsü (Gerçek IMU verisini alıp Gazebo 3D Dronuna iletir)
    real_to_sim_node = Node(
        package='stm32_drone_control',
        executable='real_to_sim_bridge',
        name='real_to_sim_bridge',
        output='screen'
    )

    # 3. PID Tuner Düğümü
    pid_tuner_node = Node(
        package='stm32_drone_control',
        executable='pid_tuner_node',
        name='pid_tuner_node',
        output='screen',
        parameters=[pid_params_path]
    )

    # 4. RViz2 Düğümü
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # 5. Gazebo Simülasyon Dünyası Launch (Robot State Publisher + Gazebo + Pose Bridge)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        serial_bridge_node,
        real_to_sim_node,
        pid_tuner_node,
        rviz_node,
        gazebo_launch
    ])
