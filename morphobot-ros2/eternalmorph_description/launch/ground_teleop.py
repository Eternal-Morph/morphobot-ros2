#!/usr/bin/env python3
# ==============================================================================
# MorphoBot (Eternalmorph) - Kara Modu Teleop Kontrolcüsü (Ground Teleop)
# ==============================================================================
# - Aynı anda W+D (veya W+A) basarak araba gibi viraj alma desteği.
# - Alternatif tek tuşla viraj tuşları: Q (Sol İleri Viraj), E (Sağ İleri Viraj).
# - 4WD DiffDrive üzerinden fiziksel tekerlek sürüşü.
# ==============================================================================

import os
import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from actuator_msgs.msg import Actuators

HELP_MSG = """
====================================================================
🚗 ETERNALMORPH KARA MODU TELEOP KONTROLCÜSÜ (Ground Mode)
====================================================================
[İLERİ / GERİ & DÖNÜŞ - KOMBİNE SÜRÜŞ DESTEĞİ]
  W / Yukarı Ok / I   : 🚀 İleri Sür
  S / Aşağı Ok  / K   : 🔙 Geri Sür
  A / Sol Ok    / J   : 🔄 Sola Dönüş
  D / Sağ Ok    / L   : 🔄 Sağa Dönüş

[AYNI ANDA ÇİFT TUŞLA VİRAJ ALMA]
  W + D               : ↱ İleri giderken Sağa Viraj Al
  W + A               : ↰ İleri giderken Sola Viraj Al
  S + D               : ↳ Geri giderken Sağa Viraj Al
  S + A               : ↲ Geri giderken Sola Viraj Al

[ALTERNATİF TEK TUŞLA VİRAJ TUŞLARI]
  E                   : ↱ İleri Sağa Viraj (W+D kısayolu)
  Q                   : ↰ İleri Sola Viraj (W+A kısayolu)
  C                   : ↳ Geri Sağa Viraj (S+D kısayolu)
  Z                   : ↲ Geri Sola Viraj (S+A kısayolu)

[HIZ SEVİYESİ & TURBO]
  Shift + Tuşlar      : ⚡ TURBO Sürüş / Dönüş (1.5x Hız)
  1 - 5               : Hız Kademesi (Varsayılan: 3)
  SPACE (Boşluk) / X  : 🛑 ANLIK FREN & DURDUR

[MOD SEÇİMİ]
  M                   : 🦿 Kolları KARA MODUNA (Ground Mode) Katla
  N                   : ✈️  Kolları HAVA MODUNA (Air Mode) Aç
  ESC / Ctrl+C        : Çıkış
====================================================================
"""

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

AIR_MODE_TARGETS = {
    'sol_on_ground_mode': 1.5708,
    'sol_on_air_mode': -1.5708,
    'sag_on_ground_mode': -1.5708,
    'sag_on_air_mode': 1.5708,
    'sol_arka_ground_mode': -1.5708,
    'sol_arka_air_mode': 1.5708,
    'sag_arka_ground_mode': 1.5708,
    'sag_arka_air_mode': -1.5708
}


class GroundTeleop(Node):
    def __init__(self):
        super().__init__('ground_teleop_node')

        # DiffDrive /cmd_vel konusu ve ros2_control diff_drive_controller konusu
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.diff_cmd_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        # Motor publisher (Pervaneleri kapalı tutmak için)
        self.motor_pub = self.create_publisher(
            Actuators,
            '/eternalmorph/command/motor_speed',
            10
        )

        # Kol eklemleri publisher'ları
        self.arm_pubs = {}
        for joint in GROUND_MODE_TARGETS.keys():
            self.arm_pubs[joint] = self.create_publisher(
                Float64,
                f'/eternalmorph/{joint}/cmd_pos',
                10
            )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )

        self.has_odom = False
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.ang_vel_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        # Bağımsız iki eksen durumu
        self.forward_command_dir = 0.0
        self.forward_fast = False
        self.last_forward_time = 0.0

        self.yaw_command_dir = 0.0
        self.yaw_fast = False
        self.last_yaw_time = 0.0

        # Kesintisiz tuş basma penceresi
        self.COMMAND_TIMEOUT = 0.45  # saniye

        # Filtrelenmiş hızlar
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

        self.speed_level = 3
        self.speed_scales = {1: 0.5, 2: 0.75, 3: 1.0, 4: 1.3, 5: 1.6}

        self.last_time = self.get_clock().now()
        self.is_ground_mode = False

    def odom_callback(self, msg: Odometry):
        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z
        self.ang_vel_z = msg.twist.twist.angular.z
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.has_odom = True

    def trigger_ground_mode(self):
        msg = Float64()
        for joint, pos in GROUND_MODE_TARGETS.items():
            msg.data = float(pos)
            self.arm_pubs[joint].publish(msg)
        self.is_ground_mode = True

    def trigger_air_mode(self):
        msg = Float64()
        for joint, pos in AIR_MODE_TARGETS.items():
            msg.data = float(pos)
            self.arm_pubs[joint].publish(msg)
        self.is_ground_mode = False

    def compute_controls(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            dt = 0.02

        now_sec = time.time()
        scale = self.speed_scales.get(self.speed_level, 1.0)

        # 1. İLERİ / GERİ
        is_forward_active = (now_sec - self.last_forward_time) < self.COMMAND_TIMEOUT
        if is_forward_active:
            base_speed = 2.2 if self.forward_fast else 1.4
            desired_vx = self.forward_command_dir * base_speed * scale
        else:
            desired_vx = 0.0
            self.forward_command_dir = 0.0

        # 2. DÖNÜŞ (YAW)
        is_yaw_active = (now_sec - self.last_yaw_time) < self.COMMAND_TIMEOUT
        if is_yaw_active:
            base_yaw = 2.8 if self.yaw_fast else 1.8
            desired_wz = self.yaw_command_dir * base_yaw * scale
            # Sadece dönüş yapılıyorsa (W basılmıyorsa) yerinde dönsün:
            if not is_forward_active:
                self.target_linear_x = 0.0
                desired_vx = 0.0
        else:
            desired_wz = 0.0
            self.yaw_command_dir = 0.0

        # Pürüzsüz üstel yumuşatma filtresi
        smoothing_lin = min(1.0, 10.0 * dt)
        smoothing_ang = min(1.0, 12.0 * dt)

        self.target_linear_x += smoothing_lin * (desired_vx - self.target_linear_x)
        self.target_angular_z += smoothing_ang * (desired_wz - self.target_angular_z)

        # DiffDrive motor komutu (/cmd_vel ve ros2_control)
        twist = Twist()
        twist.linear.x = float(self.target_linear_x)
        twist.angular.z = float(self.target_angular_z)
        self.cmd_vel_pub.publish(twist)
        self.diff_cmd_pub.publish(twist)

        # Pervaneler 0 RPM
        motor_msg = Actuators()
        motor_msg.header.stamp = now.to_msg()
        motor_msg.velocity = [0.0, 0.0, 0.0, 0.0]
        self.motor_pub.publish(motor_msg)

        return self.target_linear_x, self.target_angular_z

    def emergency_stop(self):
        self.forward_command_dir = 0.0
        self.yaw_command_dir = 0.0
        self.last_forward_time = 0.0
        self.last_yaw_time = 0.0
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.cmd_vel_pub.publish(Twist())


def read_keys_buffer(timeout=0.015):
    """Terminal tamponundaki tüm tuşları tek seferde okur (aynı anda W+D algılamak için)."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return ""
    try:
        return os.read(sys.stdin.fileno(), 1024).decode('utf-8', errors='ignore')
    except Exception:
        return ""


def main():
    rclpy.init()
    node = GroundTeleop()
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    print(HELP_MSG)
    status_msg = "Hazır. M tuşuna basarak KARA MODUNA geçebilirsiniz."

    try:
        while rclpy.ok():
            chars = read_keys_buffer(timeout=0.015)

            if chars:
                now_t = time.time()

                # Çıkış
                if '\x03' in chars or '\x1b\x1b' in chars:
                    break

                # Acil Durdurma
                if ' ' in chars or 'x' in chars or 'X' in chars:
                    node.emergency_stop()
                    status_msg = "🛑 ANLIK DURDUR"

                # Mod Değişimi
                if 'm' in chars or 'M' in chars:
                    node.trigger_ground_mode()
                    status_msg = "🦿 KARA MODUNA ALINDI!"
                elif 'n' in chars or 'N' in chars:
                    node.trigger_air_mode()
                    status_msg = "✈️ HAVA MODUNA ALINDI!"

                # Hız Kademeleri
                for k in ['1', '2', '3', '4', '5']:
                    if k in chars:
                        node.speed_level = int(k)
                        status_msg = f"⚙️ Hız: {node.speed_level}"

                # Turbo Kontrolü (Shift ile basılan büyük harfler)
                is_shift = any(c in chars for c in ['W', 'S', 'A', 'D', 'E', 'Q'])

                # --- 1. İLERİ / GERİ TUŞLARI ---
                has_forward = ('w' in chars or 'W' in chars or '\x1b[A' in chars or 'i' in chars or 'I' in chars or 'e' in chars or 'E' in chars or 'q' in chars or 'Q' in chars)
                has_backward = ('s' in chars or 'S' in chars or '\x1b[B' in chars or 'k' in chars or 'K' in chars or 'c' in chars or 'C' in chars or 'z' in chars or 'Z' in chars)

                if has_forward and not has_backward:
                    node.forward_command_dir = 1.0
                    node.forward_fast = is_shift
                    node.last_forward_time = now_t
                elif has_backward and not has_forward:
                    node.forward_command_dir = -1.0
                    node.forward_fast = is_shift
                    node.last_forward_time = now_t

                # --- 2. DÖNÜŞ (YAW) TUŞLARI ---
                has_left = ('a' in chars or 'A' in chars or '\x1b[D' in chars or 'j' in chars or 'J' in chars or 'q' in chars or 'Q' in chars or 'z' in chars or 'Z' in chars)
                has_right = ('d' in chars or 'D' in chars or '\x1b[C' in chars or 'l' in chars or 'L' in chars or 'e' in chars or 'E' in chars or 'c' in chars or 'C' in chars)

                if has_left and not has_right:
                    node.yaw_command_dir = 1.0
                    node.yaw_fast = is_shift
                    node.last_yaw_time = now_t
                elif has_right and not has_left:
                    node.yaw_command_dir = -1.0
                    node.yaw_fast = is_shift
                    node.last_yaw_time = now_t

                # Durum mesajı
                if has_forward and has_right:
                    status_msg = "🚀↱ İleri + Sağ Viraj (W+D)"
                elif has_forward and has_left:
                    status_msg = "🚀↰ İleri + Sol Viraj (W+A)"
                elif has_backward and has_right:
                    status_msg = "🔙↳ Geri + Sağ Viraj (S+D)"
                elif has_backward and has_left:
                    status_msg = "🔙↲ Geri + Sol Viraj (S+A)"
                elif has_forward:
                    status_msg = "⚡ TURBO İleri" if is_shift else "🚀 İleri"
                elif has_backward:
                    status_msg = "⚡ TURBO Geri" if is_shift else "🔙 Geri"
                elif has_left:
                    status_msg = "⚡ TURBO Sola Dönüş" if is_shift else "🔄 Sola Dönüş"
                elif has_right:
                    status_msg = "⚡ TURBO Sağa Dönüş" if is_shift else "🔄 Sağa Dönüş"

            rclpy.spin_once(node, timeout_sec=0.005)
            cur_vx, cur_wz = node.compute_controls()

            mode_tag = "KARA" if node.is_ground_mode else "SERBEST"
            spd = node.speed_level
            odom_str = f"| Odom: Vx:{node.vel_x:+4.1f}m/s Wz:{node.ang_vel_z:+4.1f}rad/s" if node.has_odom else ""

            sys.stdout.write(
                f"\r[{mode_tag}|HIZ:{spd}] İleri:{cur_vx:+4.2f}m/s Dönüş:{cur_wz:+4.2f}rad/s {odom_str} | {status_msg:<28}"
            )
            sys.stdout.flush()

    except Exception as e:
        print(f"\nHata: {e}")
    finally:
        node.emergency_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n\nGround Teleop sonlandırıldı. Motorlar kapatıldı.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
