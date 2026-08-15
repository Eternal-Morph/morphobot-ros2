import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stm32_drone_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emirhan',
    maintainer_email='emirhan@todo.todo',
    description='ROS 2 bridge and visualization package for STM32 H7 MPU6050 drone controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_imu_bridge = stm32_drone_control.serial_imu_bridge:main',
            'pid_tuner_node = stm32_drone_control.pid_tuner_node:main',
            'real_to_sim_bridge = stm32_drone_control.real_to_sim_bridge:main',
        ],
    },
)
