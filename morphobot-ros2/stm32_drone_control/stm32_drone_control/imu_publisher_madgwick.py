import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import serial
import math

# Bağımlılık gerektirmeyen, saf matematiksel Euler -> Quaternion dönüşümü
def euler_to_quaternion(roll, pitch, yaw):
    # ROS (ENU) ve Sensör (NED) eksen uyuşmazlığını gidermek için
    # Eğer eksenlerden biri ters hareket ediyorsa buradaki eksi (-) işaretlerini değiştirebilirsin.
    r = -roll
    p = pitch
    y = yaw

    qx = math.sin(r/2) * math.cos(p/2) * math.cos(y/2) - math.cos(r/2) * math.sin(p/2) * math.sin(y/2)
    qy = math.cos(r/2) * math.sin(p/2) * math.cos(y/2) + math.sin(r/2) * math.cos(p/2) * math.sin(y/2)
    qz = math.cos(r/2) * math.cos(p/2) * math.sin(y/2) - math.sin(r/2) * math.sin(p/2) * math.cos(y/2)
    qw = math.cos(r/2) * math.cos(p/2) * math.cos(y/2) + math.sin(r/2) * math.sin(p/2) * math.sin(y/2)
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
            if hasattr(self, 'ser') and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                if not line:
                    return
                
                parts = line.split('|')
                
                # FORMAT: "R: 12.34 | P: 45.67 | Y: 89.01 | CT: 10"
                if len(parts) >= 3:
                    try:
                        # --- ÇÖZÜM: ROLL VE PITCH YER DEĞİŞTİRDİ ---
                        # Sensörden ilk gelen (0. indeks) aslında Pitch ekseniymiş, onu düzelttik.
                        pitch_deg = float(parts[0].split(':')[1].strip())
                        roll_deg  = float(parts[1].split(':')[1].strip())
                        yaw_deg   = float(parts[2].split(':')[1].strip())
                        
                        roll_rad  = math.radians(roll_deg)
                        pitch_rad = math.radians(pitch_deg)
                        yaw_rad   = math.radians(yaw_deg)
                        
                        # Euler'den Quaternion'a dönüştür
                        q = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                        
                        # 1. TF Yayını (world -> base_link)
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
                        
                        # Küp boyutları (X ekseni yani 'Ön' tarafı 0.6 ile daha uzun, neyin ön olduğunu anlarsın)
                        marker.scale.x = 0.6  
                        marker.scale.y = 0.3  
                        marker.scale.z = 0.1  
                        
                        # Renk (Yarı saydam yeşil)
                        marker.color.a = 0.8
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        
                        self.marker_pub.publish(marker)
                        
                    except (IndexError, ValueError):
                        # Gürültülü/Yarım gelen seri verileri es geç
                        pass
                    
        except Exception as e:
            self.get_logger().error(f"Beklenmeyen hata: {e}")

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
