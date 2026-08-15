import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('stm32_drone_control')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Gazebo 3D Mesh Yolları
    pkg_parent_share = os.path.dirname(pkg_share)
    ign_resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        pkg_parent_share + (':' + os.environ['IGN_GAZEBO_RESOURCE_PATH'] if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ else '')
    )
    gz_resource_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        pkg_parent_share + (':' + os.environ['GZ_SIM_RESOURCE_PATH'] if 'GZ_SIM_RESOURCE_PATH' in os.environ else '')
    )

    # URDF ve Robot Description
    urdf_path = os.path.join(pkg_share, 'urdf', 'drone.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo Simülasyonu Başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Drone Modeline Gazebo'da Nesne Oluştur (Spawn - URDF robot_description)
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

    # ROS 2 -> Gazebo Harmonic Set Pose Köprüsü (/world/empty/set_pose)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
        ],
        output='screen'
    )

    return LaunchDescription([
        ign_resource_env,
        gz_resource_env,
        rsp_node,
        gazebo,
        spawn_drone,
        bridge
    ])
