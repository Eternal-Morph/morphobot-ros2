#!/usr/bin/env python3
# ==============================================================================
# MorphoBot (Eternalmorph) - Kapalı Çevrim PID Uçuş Kontrolcüsü
# ==============================================================================

import sys
import math
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators
from std_msgs.msg import Float64

HELP_MSG = """
====================================================================
🚁 ETERNALMORPH SAKİN & HASSAS PID UÇUŞ KONTROLCÜSÜ
====================================================================
[GAZ (THROTTLE) & İRTİFA - ÇOK HASSAS KONTROL]
  H              : 🎯 HOVER KİLİDİ (1226 rad/s - Havada sabit asılı kalır)
  w / s          : Yavaş Yüksel / Alçal (±5 rad/s - Sakin ve kontrollü)
  Shift + W / S  : Hızlı Yüksel / Alçal (±25 rad/s)
  SPACE (Boşluk) : 🛑 ACİL MOTOR DURDUR (Gaz = 0)

[YÖN KONTROLÜ - YAYLI & OTOMATİK DÜZELEN (AUTO-LEVEL)]
  Yukarı Ok / I   : İleri Süzül (Tuş basılıyken hafif eğilir, bırakınca düzleşip durur)
  Aşağı Ok  / K   : Geri Süzül  (Tuş basılıyken hafif eğilir, bırakınca düzleşip durur)
  Sol Ok    / J   : Sola Süzül  (Tuş basılıyken hafif yatar, bırakınca düzleşip durur)
  Sağ Ok    / L   : Sağa Süzül  (Tuş basılıyken hafif yatar, bırakınca düzleşip durur)
  X               : Tam Dur & Hover (Açıları sıfırlar ve 1226 rad/s hover'a kilitler)

[DÖNÜŞ (YAW)]
  A / D          : Sola / Sağa Dönüş (Tuşu bırakınca durur)

[DİĞER]
  P              : PID Dengeleyiciyi Aç / Kapat
  M              : Kolları Otomatik HAVA MODUNA (Air Mode) Al
  Q              : Çıkış ve Motorları Güvenli Kapat
====================================================================
"""

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


def quat_to_euler_xyz(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    theta_x = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    theta_y = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    theta_z = math.atan2(siny_cosp, cosy_cosp)

    return theta_x, theta_y, theta_z


class FlightPIDTeleop(Node):
    def __init__(self):
        super().__init__('flight_pid_teleop_node')

        self.motor_pub = self.create_publisher(
            Actuators,
            '/eternalmorph/command/motor_speed',
            10
        )

        self.arm_pubs = {}
        for joint in AIR_MODE_TARGETS.keys():
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
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/sim_data',
            self.imu_callback,
            sensor_qos
        )

        self.hover_throttle = 1226.0  # Tam 0 ivme asılı kalma hızı (12.259 kg @ 2e-5 Ct)
        self.target_throttle = 0.0
        self.actual_throttle = 0.0
        self.max_motor_vel = 2000.0  # rad/s
        self.pid_enabled = True

        self.target_pitch = 0.0
        self.target_roll = 0.0
        self.target_yaw_rate = 0.0

        # Yaylı tuş komutları (tuş basılıyken hafifçe eğilir/döner, bırakınca kendiliğinden sıfırlanır)
        self.pitch_command_dir = 0.0
        self.last_pitch_time = 0.0
        self.roll_command_dir = 0.0
        self.last_roll_time = 0.0
        self.yaw_command_dir = 0.0
        self.last_yaw_time = 0.0

        self.has_imu = False
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        # Eksen eylemsizliklerine göre (I_xx=0.34, I_yy=0.21) ayrı ayrı optimize edilmiş kazançlar
        self.kp_pitch = 120.0
        self.kd_pitch = 38.0
        self.ki_pitch = 0.5

        # Roll ekseni eylemsizliği daha küçük olduğu için kazançlar daha yumuşak (sağa-sola sallanmayı keser)
        self.kp_roll = 75.0
        self.kd_roll = 24.0
        self.ki_roll = 0.2

        self.kp_yaw = 50.0

        self.integral_pitch = 0.0
        self.integral_roll = 0.0
        self.last_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        pitch, roll, yaw = quat_to_euler_xyz(q.x, q.y, q.z, q.w)
        self.current_pitch = pitch
        self.current_roll = roll
        self.current_yaw = yaw

        # Low-pass filter (Gürültü ve simülasyon titreşimini önler)
        alpha = 0.35
        self.gyro_x = (1.0 - alpha) * self.gyro_x + alpha * msg.angular_velocity.x
        self.gyro_y = (1.0 - alpha) * self.gyro_y + alpha * msg.angular_velocity.y
        self.gyro_z = (1.0 - alpha) * self.gyro_z + alpha * msg.angular_velocity.z
        self.has_imu = True

    def trigger_air_mode(self):
        msg = Float64()
        for joint, pos in AIR_MODE_TARGETS.items():
            msg.data = float(pos)
            self.arm_pubs[joint].publish(msg)

    def compute_controls(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            dt = 0.02

        if self.target_throttle <= 0.0:
            self.actual_throttle = 0.0
            self.integral_pitch = 0.0
            self.integral_roll = 0.0
            self.target_yaw_rate = 0.0
            return 0.0, 0.0, 0.0, 0.0

        # Gaz geçişini yumuşat (ani sıçramayı ve sendelemeyi engelle)
        ramp_speed = 80.0  # rad/s per saniye (ani sarsıntıyı önler)
        if self.actual_throttle < self.target_throttle:
            self.actual_throttle = min(self.target_throttle, self.actual_throttle + ramp_speed * dt)
        elif self.actual_throttle > self.target_throttle:
            self.actual_throttle = max(self.target_throttle, self.actual_throttle - ramp_speed * dt)

        if self.actual_throttle < 1150.0:
            self.integral_pitch = 0.0
            self.integral_roll = 0.0

        now_sec = time.time()

        # İleri / Geri (Pitch) - Tuş basılıyken hafifçe öne/arkaya eğilir (~3.5°), bırakılınca kendiliğinden düzleşir
        MAX_PITCH_ANGLE = math.radians(3.5)  # 3.5 derece - sakin ve yumuşak süzülme
        if (now_sec - self.last_pitch_time) < 0.28:
            desired_pitch = self.pitch_command_dir * MAX_PITCH_ANGLE
        else:
            desired_pitch = 0.0
            self.pitch_command_dir = 0.0

        # Sağ / Sol (Roll) - Tuş basılıyken hafifçe yatar (~3.5°), bırakılınca kendiliğinden düzleşir
        MAX_ROLL_ANGLE = math.radians(3.5)   # 3.5 derece - sakin ve yumuşak süzülme
        if (now_sec - self.last_roll_time) < 0.28:
            desired_roll = self.roll_command_dir * MAX_ROLL_ANGLE
        else:
            desired_roll = 0.0
            self.roll_command_dir = 0.0

        # Dönüş (Yaw) komutu - Tuş basılıyken döner, bırakılınca durur
        if (now_sec - self.last_yaw_time) < 0.28:
            desired_yaw = self.yaw_command_dir * 0.40  # rad/s (~23 deg/s)
        else:
            desired_yaw = 0.0
            self.yaw_command_dir = 0.0

        # Pürüzsüz açı ve hız rampası (sarsıntısız yumuşak geçiş)
        angle_smoothing = min(1.0, 6.0 * dt)
        self.target_pitch += angle_smoothing * (desired_pitch - self.target_pitch)
        self.target_roll += angle_smoothing * (desired_roll - self.target_roll)

        rate_smoothing = min(1.0, 8.0 * dt)
        self.target_yaw_rate += rate_smoothing * (desired_yaw - self.target_yaw_rate)

        u_pitch = 0.0
        u_roll = 0.0
        u_yaw = 0.0

        if self.pid_enabled and self.has_imu:
            # Yüksek hızda itki karesel arttığı için kazancı dinamik olarak yumuşat (yüksek hızda sallanmayı keser)
            gain_scale = (1230.0 / max(1000.0, self.actual_throttle)) ** 1.5
            cur_kp_p = self.kp_pitch * gain_scale
            cur_kd_p = self.kd_pitch * gain_scale
            cur_ki_p = self.ki_pitch * gain_scale

            cur_kp_r = self.kp_roll * gain_scale
            cur_kd_r = self.kd_roll * gain_scale
            cur_ki_r = self.ki_roll * gain_scale

            # Pitch PID (Burun Dengeleme)
            err_pitch = self.target_pitch - self.current_pitch
            if self.actual_throttle >= 1150.0:
                self.integral_pitch += err_pitch * dt
                self.integral_pitch = max(-0.10, min(0.10, self.integral_pitch))

            u_pitch = (cur_kp_p * err_pitch +
                       cur_ki_p * self.integral_pitch -
                       cur_kd_p * self.gyro_x)

            # Roll PID (Sağ/Sol Dengeleme)
            err_roll = self.target_roll - self.current_roll
            if self.actual_throttle >= 1150.0:
                self.integral_roll += err_roll * dt
                self.integral_roll = max(-0.10, min(0.10, self.integral_roll))

            u_roll = (cur_kp_r * err_roll +
                      cur_ki_r * self.integral_roll -
                      cur_kd_r * self.gyro_y)

            # Yaw Kontrolü (Pürüzsüz hız takibi)
            err_yaw_rate = self.target_yaw_rate - self.gyro_z
            u_yaw = self.kp_yaw * gain_scale * err_yaw_rate

            # Sallantıyı tamamen kesen dengeli limitler
            u_pitch = max(-50.0, min(50.0, u_pitch))
            u_roll = max(-35.0, min(35.0, u_roll))
            u_yaw = max(-45.0, min(45.0, u_yaw))

        # Motor Mikseri:
        # Motor 0 (Sol Ön - FL, CCW):   T + Pitch + Roll - Yaw
        # Motor 1 (Sol Arka - RL, CW):  T - Pitch + Roll + Yaw
        # Motor 2 (Sağ Ön - FR, CW):    T + Pitch - Roll + Yaw
        # Motor 3 (Sağ Arka - RR, CCW): T - Pitch - Roll - Yaw
        m0 = self.actual_throttle + u_pitch + u_roll - u_yaw
        m1 = self.actual_throttle - u_pitch + u_roll + u_yaw
        m2 = self.actual_throttle + u_pitch - u_roll + u_yaw
        m3 = self.actual_throttle - u_pitch - u_roll - u_yaw

        m0 = max(0.0, min(self.max_motor_vel, m0))
        m1 = max(0.0, min(self.max_motor_vel, m1))
        m2 = max(0.0, min(self.max_motor_vel, m2))
        m3 = max(0.0, min(self.max_motor_vel, m3))

        msg = Actuators()
        msg.header.stamp = now.to_msg()
        msg.velocity = [float(m0), float(m1), float(m2), float(m3)]
        self.motor_pub.publish(msg)

        return m0, m1, m2, m3


def get_key(settings, timeout=0.015):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            extra = sys.stdin.read(2)
            key += extra
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = FlightPIDTeleop()
    settings = termios.tcgetattr(sys.stdin)

    print(HELP_MSG)

    status_msg = "Hazır. Motorlar kapalı."

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.015)

            if key:
                if key in ['h', 'H']:
                    node.target_throttle = node.hover_throttle
                    if node.actual_throttle < 1000.0:
                        node.actual_throttle = 1160.0  # Yumuşak zemin kalkışı başlat
                    node.target_pitch = 0.0
                    node.target_roll = 0.0
                    node.target_yaw_rate = 0.0
                    node.pitch_command_dir = 0.0
                    node.roll_command_dir = 0.0
                    node.yaw_command_dir = 0.0
                    node.integral_pitch = 0.0
                    node.integral_roll = 0.0
                    status_msg = f"🎯 HOVER KİLİDİ ({node.hover_throttle:.0f} rad/s)"
                elif key in ['w', 'W']:
                    step = 25.0 if key == 'W' else 5.0
                    node.target_throttle = min(node.max_motor_vel, node.target_throttle + step)
                    status_msg = f"Gaz Artır: {node.target_throttle:.0f} (+{step:.0f})"
                elif key in ['s', 'S']:
                    step = 25.0 if key == 'S' else 5.0
                    node.target_throttle = max(0.0, node.target_throttle - step)
                    status_msg = f"Gaz Azalt: {node.target_throttle:.0f} (-{step:.0f})"
                elif key == ' ':
                    node.target_throttle = 0.0
                    node.actual_throttle = 0.0
                    node.target_pitch = 0.0
                    node.target_roll = 0.0
                    node.target_yaw_rate = 0.0
                    node.pitch_command_dir = 0.0
                    node.roll_command_dir = 0.0
                    node.yaw_command_dir = 0.0
                    node.integral_pitch = 0.0
                    node.integral_roll = 0.0
                    status_msg = "🛑 MOTORLAR DURDURULDU!"
                elif key in ['\x1b[A', 'i', 'I']:  # Up Arrow
                    node.pitch_command_dir = -1.0
                    node.last_pitch_time = time.time()
                    status_msg = "İleri Süzül"
                elif key in ['\x1b[B', 'k', 'K']:  # Down Arrow
                    node.pitch_command_dir = 1.0
                    node.last_pitch_time = time.time()
                    status_msg = "Geri Süzül"
                elif key in ['\x1b[D', 'j', 'J']:  # Left Arrow
                    node.roll_command_dir = -1.0
                    node.last_roll_time = time.time()
                    status_msg = "Sola Süzül"
                elif key in ['\x1b[C', 'l', 'L']:  # Right Arrow
                    node.roll_command_dir = 1.0
                    node.last_roll_time = time.time()
                    status_msg = "Sağa Süzül"
                elif key in ['a', 'A']:
                    node.yaw_command_dir = 1.0
                    node.last_yaw_time = time.time()
                    status_msg = "Sola Dönüş (Yaw)"
                elif key in ['d', 'D']:
                    node.yaw_command_dir = -1.0
                    node.last_yaw_time = time.time()
                    status_msg = "Sağa Dönüş (Yaw)"
                elif key in ['x', 'X']:
                    node.pitch_command_dir = 0.0
                    node.roll_command_dir = 0.0
                    node.yaw_command_dir = 0.0
                    node.target_pitch = 0.0
                    node.target_roll = 0.0
                    node.target_yaw_rate = 0.0
                    node.target_throttle = node.hover_throttle
                    status_msg = f"Durdur & Hover ({node.hover_throttle:.0f})"
                elif key in ['p', 'P']:
                    node.pid_enabled = not node.pid_enabled
                    state = "AÇIK" if node.pid_enabled else "KAPALI"
                    status_msg = f"PID Dengeleyici: {state}"
                elif key in ['m', 'M']:
                    node.trigger_air_mode()
                    status_msg = "✈️ Kollar HAVA MODUNA ayarlandı!"
                elif key in ['q', 'Q', '\x03']:
                    break

            rclpy.spin_once(node, timeout_sec=0.005)
            m0, m1, m2, m3 = node.compute_controls()

            # Canlı Telemetri
            pid_tag = "PID:ON " if node.pid_enabled else "PID:OFF"
            imu_tag = "IMU:OK" if node.has_imu else "IMU:BEK"
            cur_p_deg = math.degrees(node.current_pitch)
            cur_r_deg = math.degrees(node.current_roll)

            sys.stdout.write(
                f"\r[{pid_tag}|{imu_tag}] GAZ:{node.actual_throttle:4.0f}/{node.target_throttle:4.0f} | "
                f"Açı: P:{cur_p_deg:+4.1f}° R:{cur_r_deg:+4.1f}° | "
                f"M:[{m0:.0f}, {m1:.0f}, {m2:.0f}, {m3:.0f}] | {status_msg:<32}"
            )
            sys.stdout.flush()

    except Exception as e:
        print(f"\nHata: {e}")
    finally:
        node.target_throttle = 0.0
        node.actual_throttle = 0.0
        node.compute_controls()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n\nMotorlar kapatıldı. Kontrolcü sonlandırıldı.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
