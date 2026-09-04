#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

class RealToSimBridge(Node):
    """
    Gerçek STM32 IMU verisini alıp Gazebo Harmonic 'set_pose' servisine 
    canlı olarak aktaran HIL Servis Köprüsü.
    """
    def __init__(self):
        super().__init__('real_to_sim_bridge')

        self.client = self.create_client(
            SetEntityPose,
            '/world/empty/set_pose'
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.pending_future = None
        self.get_logger().info("HIL Real-to-Sim Gazebo Servis Köprüsü Başlatıldı!")

    def imu_callback(self, msg: Imu):
        if not self.client.service_is_ready():
            return

        # Bekleyen bir servis isteği varsa yenisiyle boğma (Gazebo henüz açılırken log birikmesini engeller)
        if self.pending_future is not None and not self.pending_future.done():
            return

        req = SetEntityPose.Request()
        req.entity.name = 'stm32_drone'
        req.entity.type = Entity.MODEL

        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = 0.5

        req.pose.orientation.x = msg.orientation.x
        req.pose.orientation.y = msg.orientation.y
        req.pose.orientation.z = msg.orientation.z
        req.pose.orientation.w = msg.orientation.w

        self.pending_future = self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = RealToSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

