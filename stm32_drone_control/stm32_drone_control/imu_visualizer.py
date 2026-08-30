import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import serial
import math

# Euler (Derece) -> Quaternion Dönüşüm Fonksiyonu
def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        
        # TF ve Marker yayınlayıcılarını başlat
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'imu_box', 10)
        
        # Seri Port Bağlantısı
        SERIAL_PORT = '/dev/ttyACM0'
        BAUD_RATE = 115200
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"{SERIAL_PORT} portu dinleniyor ve RViz'e veri basılıyor...")
        except Exception as e:
            self.get_logger().error(f"Seri port açılamadı ({SERIAL_PORT}): {e}")
            raise e
        
        # Döngü Timer'ı (50ms'de bir çalışır)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                if not line:
                    return
                
                parts = line.split('|')
                
                # En az 2 parça varsa (Pitch ve Roll) al ve işle
                if len(parts) >= 2:
                    try:
                        pitch_deg = float(parts[0].split(':')[1].strip())
                        roll_deg  = float(parts[1].split(':')[1].strip())
                        
                        pitch_rad = math.radians(pitch_deg)
                        roll_rad  = math.radians(roll_deg)
                        yaw_rad   = 0.0
                        
                        q = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                        
                        # 1. TF Yayını
                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
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
                        
                        # 2. RViz Marker Yayını
                        marker = Marker()
                        marker.header.frame_id = 'base_link'
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.type = Marker.CUBE
                        marker.action = Marker.ADD
                        
                        marker.scale.x = 0.5
                        marker.scale.y = 0.3
                        marker.scale.z = 0.1
                        
                        marker.color.a = 1.0
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        
                        self.marker_pub.publish(marker)
                        
                    except (IndexError, ValueError) as parse_err:
                        self.get_logger().warn(f"Ayrıştırma hatası: {parse_err} | Gelen: '{line}'")
                else:
                    self.get_logger().warn(f"Eksik parça: {line}")
                    
        except Exception as e:
            self.get_logger().error(f"Hata: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
