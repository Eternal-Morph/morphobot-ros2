#!/usr/bin/env python3
import time
import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDriver(Node):
    def __init__(self, diameter=10.0, linear_speed=1.0):
        super().__init__('circle_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        radius = diameter / 2.0
        angular_speed = linear_speed / radius  # v = w * r  =>  w = v / r
        circumference = math.pi * diameter
        total_time = circumference / linear_speed  # 1 tam tur suresi

        self.get_logger().info(f"Daire Cizimi Baslatiliyor:")
        self.get_logger().info(f" - Cap: {diameter} m (Yaricap: {radius} m)")
        self.get_logger().info(f" - Cizgisel Hiz (v): {linear_speed} m/s")
        self.get_logger().info(f" - Acisal Hiz (w): {angular_speed:.3f} rad/s")
        self.get_logger().info(f" - 1 Tam Tur Suresi: {total_time:.2f} saniye")

        # 1 saniye bekle (publisher hazir olsun)
        time.sleep(1.0)

        # Hiz komutu
        twist = Twist()
        twist.linear.x = float(linear_speed)
        twist.angular.z = float(angular_speed)

        # Dongu
        rate_hz = 20
        dt = 1.0 / rate_hz
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < total_time:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(dt)

        # Daire bitince robotu durdur
        stop_twist = Twist()
        for _ in range(5):
            self.pub.publish(stop_twist)
            time.sleep(0.05)

        self.get_logger().info("1 tam tur tamamlandi ve robot durduruldu!")

def main(args=None):
    rclpy.init(args=args)
    
    diameter = 10.0
    speed = 1.0
    if len(sys.argv) > 1:
        try:
            diameter = float(sys.argv[1])
        except ValueError:
            pass
    if len(sys.argv) > 2:
        try:
            speed = float(sys.argv[2])
        except ValueError:
            pass

    node = CircleDriver(diameter=diameter, linear_speed=speed)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
