#!/usr/bin/env python3
# ==============================================================================
# Eternalmorph - Otomatik Kara Sürüş ve Dönüş Test Scripti
# ==============================================================================
# Sırasıyla:
# 1. Kolları Kara Moduna katlar (M)
# 2. İleri sürer
# 3. Durur
# 4. Sola döner
# 5. Durur
# 6. İleri sürer
# 7. Durur
# 8. Sağa döner
# 9. Durur
# 10. İleri + Sağa viraj alır (W+D)
# 11. İleri + Sola viraj alır (W+A)
# 12. Güvenli şekilde durup motorları kapatır.
# ==============================================================================

import time
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from actuator_msgs.msg import Actuators

GROUND_MODE_TARGETS = {
    'sol_on_ground_mode': 1.5708,
    'sol_on_air_mode': 0.0,
    'sag_on_ground_mode': -1.5708,
    'sag_on_air_mode': 0.0,
    'sol_arka_ground_mode': -1.5708,
    'sol_arka_air_mode': 0.0,
    'sag_arka_ground_mode': 1.5708,
    'sag_arka_air_mode': 0.0
}

class GroundAutoTest(Node):
    def __init__(self):
        super().__init__('ground_auto_test_node')

        # DiffDrive / ros2_control cmd publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.diff_cmd_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        # Motor publisher (Pervaneleri 0 RPM tutmak için)
        self.motor_pub = self.create_publisher(Actuators, '/eternalmorph/command/motor_speed', 10)

        # Kol katlama eklemleri
        self.arm_pubs = {}
        for joint in GROUND_MODE_TARGETS.keys():
            self.arm_pubs[joint] = self.create_publisher(Float64, f'/eternalmorph/{joint}/cmd_pos', 10)

        self.get_logger().info("Ground Auto Test Node baslatildi.")

    def set_ground_mode(self):
        """Kolları kara moduna katlar."""
        for joint, pos in GROUND_MODE_TARGETS.items():
            msg = Float64()
            msg.data = float(pos)
            self.arm_pubs[joint].publish(msg)

    def publish_twist(self, forward_speed=0.0, yaw_rate=0.0):
        """
        forward_speed > 0: İleri
        forward_speed < 0: Geri
        yaw_rate > 0: Sola Dönüş
        yaw_rate < 0: Sağa Dönüş
        """
        twist = Twist()
        # Tekerlek koordinat eksenine gore fiziksel yon duzeltmesi:
        twist.linear.x = -float(forward_speed)
        twist.angular.z = -float(yaw_rate)

        self.cmd_vel_pub.publish(twist)
        self.diff_cmd_pub.publish(twist)

        # Pervaneler kapalı
        motor_msg = Actuators()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.velocity = [0.0, 0.0, 0.0, 0.0]
        self.motor_pub.publish(motor_msg)

    def run_stage(self, name, forward_speed, yaw_rate, duration_sec):
        print(f"\n▶ [{name}] -> Hız: {forward_speed:+.2f} m/s | Dönüş: {yaw_rate:+.2f} rad/s | Süre: {duration_sec}s")
        start_time = time.time()
        while time.time() - start_time < duration_sec and rclpy.ok():
            self.publish_twist(forward_speed, yaw_rate)
            self.set_ground_mode()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def stop_robot(self, duration_sec=1.0):
        print(f"⏸ [DUR] Robot frenliyor ({duration_sec}s)...")
        start_time = time.time()
        while time.time() - start_time < duration_sec and rclpy.ok():
            self.publish_twist(0.0, 0.0)
            self.set_ground_mode()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

def main():
    rclpy.init()
    node = GroundAutoTest()

    print("\n" + "=" * 60)
    print("🤖 ETERNALMORPH OTOMATİK SÜRÜŞ VE DÖNÜŞ TESTİ")
    print("=" * 60)
    print("1. Robot Kara Moduna alınıyor (kollar katlanıyor)...")

    # Kara moduna geçip oturması için 3 saniye bekle
    for _ in range(150):
        node.set_ground_mode()
        node.publish_twist(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    print("✔ Kollar hazır! Test senaryosu başlıyor...\n")
    time.sleep(1.0)

    try:
        # 1. DÜZ İLERİ
        node.run_stage("1. DÜZ İLERİ SÜRÜŞ (W)", forward_speed=1.0, yaw_rate=0.0, duration_sec=3.0)
        node.stop_robot(duration_sec=1.0)

        # 2. YERİNDE SOLA DÖNÜŞ
        node.run_stage("2. YERİNDE SOLA DÖNÜŞ (A)", forward_speed=0.0, yaw_rate=0.75, duration_sec=2.5)
        node.stop_robot(duration_sec=1.0)

        # 3. TEKRAR DÜZ İLERİ
        node.run_stage("3. DÜZ İLERİ SÜRÜŞ (W)", forward_speed=1.0, yaw_rate=0.0, duration_sec=3.0)
        node.stop_robot(duration_sec=1.0)

        # 4. YERİNDE SAĞA DÖNÜŞ
        node.run_stage("4. YERİNDE SAĞA DÖNÜŞ (D)", forward_speed=0.0, yaw_rate=-0.75, duration_sec=2.5)
        node.stop_robot(duration_sec=1.0)

        # 5. İLERİ + SAĞA VİRAJ (W + D)
        node.run_stage("5. İLERİ + SAĞA VİRAJ (W+D)", forward_speed=0.9, yaw_rate=-0.55, duration_sec=3.5)
        node.stop_robot(duration_sec=1.0)

        # 6. İLERİ + SOLA VİRAJ (W + A)
        node.run_stage("6. İLERİ + SOLA VİRAJ (W+A)", forward_speed=0.9, yaw_rate=0.55, duration_sec=3.5)
        node.stop_robot(duration_sec=1.0)

        # 7. GERİ SÜRÜŞ (S)
        node.run_stage("7. DÜZ GERİ SÜRÜŞ (S)", forward_speed=-1.0, yaw_rate=0.0, duration_sec=2.5)
        node.stop_robot(duration_sec=1.0)

        print("\n" + "=" * 60)
        print("🎉 TEST BAŞARIYLA TAMAMLANDI! Robot güvenli durduruldu.")
        print("=" * 60 + "\n")

    except KeyboardInterrupt:
        print("\n[!] Kullanıcı testi durdurdu.")
    finally:
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
