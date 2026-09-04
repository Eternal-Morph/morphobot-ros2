#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw):
    r = -roll
    p = pitch
    y = yaw

    qx = math.sin(r/2) * math.cos(p/2) * math.cos(y/2) - math.cos(r/2) * math.sin(p/2) * math.sin(y/2)
    qy = math.cos(r/2) * math.sin(p/2) * math.cos(y/2) + math.sin(r/2) * math.cos(p/2) * math.sin(y/2)
    qz = math.cos(r/2) * math.cos(p/2) * math.sin(y/2) - math.sin(r/2) * math.sin(p/2) * math.cos(y/2)
    qw = math.cos(r/2) * math.cos(p/2) * math.cos(y/2) + math.sin(r/2) * math.sin(p/2) * math.sin(y/2)
    return [qx, qy, qz, qw]

class SerialImuBridge(Node):
    def __init__(self):
        super().__init__('serial_imu_bridge')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.marker_pub = self.create_publisher(Marker, 'imu_box', 10)
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"{self.port} portu ({self.baudrate} baud) dinleniyor...")
        except Exception as e:
            self.get_logger().error(f"Seri port acilamadi ({self.port}): {e}")
            self.ser = None

        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        if not self.ser or not self.ser.is_open:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            except Exception:
                return

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
            if not line:
                return

            parts = line.split('|')
            if len(parts) >= 3:
                roll_deg  = float(parts[0].split(':')[1].strip())
                pitch_deg = float(parts[1].split(':')[1].strip())
                yaw_deg   = float(parts[2].split(':')[1].strip())

                roll_rad  = math.radians(roll_deg)
                pitch_rad = math.radians(pitch_deg)
                yaw_rad   = math.radians(yaw_deg)

                q = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                now = self.get_clock().now().to_msg()

                # 1. IMU Data
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = 'base_link'
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
                self.imu_pub.publish(imu_msg)

                # 2. TF Yayını (world -> base_link)
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = 'world'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.tf_broadcaster.sendTransform(t)

                # 3. RViz Marker Yayını
                marker = Marker()
                marker.header.frame_id = 'base_link'
                marker.header.stamp = now
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.scale.x = 0.6  
                marker.scale.y = 0.3  
                marker.scale.z = 0.1  
                marker.color.a = 0.8
                marker.color.g = 1.0
                self.marker_pub.publish(marker)

        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialImuBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and hasattr(node, 'ser') and node.ser and node.ser.is_open:
            node.ser.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
