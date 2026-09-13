# morphobot-ros2
ROS 2 workspace, high-level control nodes, and integrated Gazebo simulations for the multi-modal hybrid drone platform.

## Prerequisites & Dependencies

- **ROS 2:** Humble Hawksbill
- **Simulator:** Gazebo Harmonic (GZ Sim)

> [!IMPORTANT]
> **Important Note:**  
> For the Gazebo Harmonic simulation and wheel controller (`gz_ros2_control` / `DiffDriveController`) to operate properly, the **`ros_gz`** and **`gz_ros2_control`** packages must be built from source specifically for **ROS 2 Humble** (branch: `humble`):
> - `ros_gz` (`humble` branch)
> - `gz_ros2_control` (`humble` branch, compiled against Gazebo Harmonic)


