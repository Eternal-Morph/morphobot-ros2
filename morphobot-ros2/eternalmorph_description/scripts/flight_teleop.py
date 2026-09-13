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
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

HELP_MSG = """
====================================================================
🚁 ETERNALMORPH SAKİN & HASSAS PID UÇUŞ KONTROLCÜSÜ (DJI MODU)
====================================================================
[GAZ (THROTTLE) & İRTİFA KONTROLÜ - YAYLI GAZ KOLU]
  H              : 🎯 HOVER KİLİDİ (Yerdeyse 1.0m kalkar, havadaysa asılı kalır)
  w (basılı tut) : 🚀 CANLI TIRMANIŞ (Tuşa bastıkça seri yükselir, bırakınca zınk diye durur)
  s (basılı tut) : 🛬 GÜVENLİ ALÇALIŞ (Tuşa bastıkça kontrollü alçalır, bırakınca durur)
  Shift + W / S  : ⚡ HIZLI Tırmanış / HIZLI Alçalış
  SPACE (Boşluk) : 🛑 ACİL MOTOR DURDUR (Gaz = 0)

[YÖN KONTROLÜ - YAYLI & ANLIK FRENLEME (AUTO-LEVEL)]
  Yukarı Ok / I   : İleri Süzül (Tuş basılıyken hafif eğilir, bırakınca düzleşir)
  Aşağı Ok  / K   : Geri Süzül / ANLIK FREN (İleri kaymayı durdurmak için 0.5 sn bas-bırak)
  Sol Ok    / J   : Sola Süzül  (Tuş basılıyken hafif yatar, bırakınca düzleşir)
  Sağ Ok    / L   : Sağa Süzül  (Tuş basılıyken hafif yatar, bırakınca düzleşir)
  X               : Tam Dur & Hover (Trimi ve açıları sıfırlar, havada sabitlenir)

[DÖNÜŞ (YAW)]
  A / D          : Sola / Sağa Dönüş (Tuşu bırakınca durur)

[TRİM AYARI (KAYMAYI DÜZELTME)]
  9 veya [       : Trimi Geriye Al (+0.2° - İleri kaymayı durdurur)
  0 veya ]       : Trimi İleriye Al (-0.2° - Geri kaymayı durdurur)

[DİĞER]
  P              : PID Dengeleyiciyi Aç / Kapat
  M              : 🦿 Kolları KARA MODUNA (Ground Mode) Al
  N              : ✈️  Kolları HAVA MODUNA (Air Mode) Al
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
        self.pos_z = 0.0
        self.target_z = 1.2  # Hedef irtifa (metre)
        self.altitude_hold_active = False

        self.hover_throttle = 1225.5  # Tam 0 ivme fiziksel denge hızı (12.2591 kg @ 2e-5 Ct, g=9.8)
        self.target_throttle = 0.0
        self.actual_throttle = 0.0
        self.max_motor_vel = 2000.0  # rad/s
        self.pid_enabled = True

        self.target_pitch = 0.0
        self.target_roll = 0.0
        self.target_yaw_rate = 0.0
        self.pitch_trim = 0.0  # radyan (Denge trimi - ileri/geri kaymayı sıfırlar)

        # Yaylı tuş komutları (tuş basılıyken hafifçe eğilir/döner, bırakınca kendiliğinden sıfırlanır)
        self.pitch_command_dir = 0.0
        self.last_pitch_time = 0.0
        self.roll_command_dir = 0.0
        self.last_roll_time = 0.0
        self.yaw_command_dir = 0.0
        self.yaw_fast = False
        self.last_yaw_time = 0.0

        # Yaylı Gaz Kolu (DJI / PX4 Tırmanış & Alçalış)
        self.climb_command_dir = 0.0
        self.climb_fast = False
        self.last_climb_time = 0.0

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
        self.ki_pitch = 12.0  # Statik burun yatıklığını ve ağırlık merkezi kaymasını yok eder

        # Roll ekseni eylemsizliği daha küçük olduğu için kazançlar daha yumuşak (sağa-sola sallanmayı keser)
        self.kp_roll = 75.0
        self.kd_roll = 24.0
        self.ki_roll = 8.0

        self.kp_yaw = 35.0  # Pürüzsüz ve sakin yaw kontrolü

        self.integral_pitch = 0.0
        self.integral_roll = 0.0
        self.last_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        # nav_msgs/Odometry REP 105 standardı: twist.twist hızları doğrudan
        # robotun gövde koordinatındadır (child_frame_id = base_link).
        # Robot gövdesi: +X Sağ, +Y İleri (Burun), +Z Yukarı
        self.vel_x = msg.twist.twist.linear.x   # Doğrudan gövde sağ/sol hızı
        self.vel_y = msg.twist.twist.linear.y   # Doğrudan gövde ileri/geri hızı
        self.vel_z = msg.twist.twist.linear.z   # Dikey hız
        self.pos_z = msg.pose.pose.position.z   # Yükseklik
        self.has_odom = True

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

    def trigger_ground_mode(self):
        msg = Float64()
        for joint, pos in GROUND_MODE_TARGETS.items():
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
        ramp_speed = 400.0  # rad/s per saniye (gecikmesiz canlı motor tepkisi)
        if self.actual_throttle < self.target_throttle:
            self.actual_throttle = min(self.target_throttle, self.actual_throttle + ramp_speed * dt)
        elif self.actual_throttle > self.target_throttle:
            self.actual_throttle = max(self.target_throttle, self.actual_throttle - ramp_speed * dt)

        if self.actual_throttle < 1150.0:
            self.integral_pitch = 0.0
            self.integral_roll = 0.0

        now_sec = time.time()
        is_yaw_active = (now_sec - self.last_yaw_time) < 0.35

        # İleri / Geri (Pitch) - Tuş basılıyken eğilir (~4.5°), bırakılınca AKTİF TERS FREN yapar
        MAX_PITCH_ANGLE = math.radians(4.5)  # 4.5 derece - akıcı ve seri yönlenme
        if (now_sec - self.last_pitch_time) < 0.28:
            desired_pitch = self.pitch_trim + self.pitch_command_dir * MAX_PITCH_ANGLE
        else:
            # Otomatik Aktif Frenleme (Yaw dönüşü sırasında frenleme kapatılır, robot dönme ekseninde düz kalır)
            auto_brake = 0.0
            if self.has_odom and self.actual_throttle >= 1150.0 and not is_yaw_active:
                if abs(self.vel_y) > 0.04:
                    auto_brake = max(-math.radians(8.5), min(math.radians(8.5), 0.22 * self.vel_y))
            desired_pitch = self.pitch_trim + auto_brake
            self.pitch_command_dir = 0.0

        # Sağ / Sol (Roll) - Tuş basılıyken yatar (~4.5°), bırakılınca AKTİF TERS FREN yapar
        MAX_ROLL_ANGLE = math.radians(4.5)   # 4.5 derece - akıcı ve seri yönlenme
        if (now_sec - self.last_roll_time) < 0.28:
            desired_roll = self.roll_command_dir * MAX_ROLL_ANGLE
        else:
            # Otomatik Aktif Frenleme (Yaw dönüşü sırasında frenleme kapatılır, robot dönme ekseninde düz kalır)
            auto_brake_r = 0.0
            if self.has_odom and self.actual_throttle >= 1150.0 and not is_yaw_active:
                if abs(self.vel_x) > 0.04:
                    auto_brake_r = -max(-math.radians(8.5), min(math.radians(8.5), 0.22 * self.vel_x))
            desired_roll = auto_brake_r
            self.roll_command_dir = 0.0

        # Dönüş (Yaw) komutu - Tuş basılıyken döner, bırakılınca durur
        if is_yaw_active:
            yaw_speed = 0.80 if self.yaw_fast else 0.45  # rad/s (~26 - 46 deg/s)
            desired_yaw = self.yaw_command_dir * yaw_speed
        else:
            desired_yaw = 0.0
            self.yaw_command_dir = 0.0

        # Pürüzsüz açı ve hız rampası (hızlı ama sarsıntısız geçiş)
        angle_smoothing = min(1.0, 10.0 * dt)
        self.target_pitch += angle_smoothing * (desired_pitch - self.target_pitch)
        self.target_roll += angle_smoothing * (desired_roll - self.target_roll)

        rate_smoothing = min(1.0, 10.0 * dt)
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
            if self.actual_throttle >= 1210.0:
                self.integral_pitch += err_pitch * dt
                self.integral_pitch = max(-1.5, min(1.5, self.integral_pitch))
            else:
                self.integral_pitch = 0.0

            u_pitch = (cur_kp_p * err_pitch +
                       cur_ki_p * self.integral_pitch -
                       cur_kd_p * self.gyro_x)

            # Roll PID (Sağ/Sol Dengeleme)
            err_roll = self.target_roll - self.current_roll
            if self.actual_throttle >= 1210.0:
                self.integral_roll += err_roll * dt
                self.integral_roll = max(-1.5, min(1.5, self.integral_roll))
            else:
                self.integral_roll = 0.0

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

        # İrtifa & Dikey Hız Kontrolü (DJI / PX4 Tarzı Yaylı Gaz Kolu):
        u_z = 0.0
        if self.actual_throttle >= 1150.0:
            is_climb_active = (now_sec - self.last_climb_time) < 0.28

            if is_climb_active and self.climb_command_dir != 0.0:
                if self.climb_command_dir > 0:
                    # W BASILI: Canlı Tırmanış
                    climb_ff = 42.0 if self.climb_fast else 24.0
                    desired_vz = 1.6 if self.climb_fast else 0.8  # m/s
                    if self.has_odom:
                        u_z = climb_ff + 15.0 * (desired_vz - self.vel_z)
                        self.target_z = self.pos_z  # Tırmanırken hedefi robotla birlikte yukarı taşı
                    else:
                        u_z = climb_ff
                else:
                    # S BASILI: Güvenli ve Kontrollü Alçalış (asla çakılmaz!)
                    descend_ff = -30.0 if self.climb_fast else -15.0
                    desired_vz = -1.2 if self.climb_fast else -0.6  # m/s
                    if self.has_odom:
                        u_z = descend_ff + 15.0 * (desired_vz - self.vel_z)
                        self.target_z = max(0.2, self.pos_z)  # Alçalırken hedefi robotla birlikte aşağı taşı
                    else:
                        u_z = descend_ff

                u_z = max(-45.0, min(70.0, u_z))
            else:
                # TUŞ BIRAKILDI / HOVER: Hedef yaylanması yok!
                # Doğrudan fiziksel denge + dikey hız söndürme (salınım ve sekme tamamen sıfır)
                self.climb_command_dir = 0.0
                if self.has_odom and self.altitude_hold_active:
                    # Sadece dikey hızı frenler, asla yay gibi geri tepmez
                    u_z = -28.0 * self.vel_z
                    u_z = max(-25.0, min(25.0, u_z))

        # Motor Mikseri:
        # Ağırlık merkezini dengeleyen ön/arka motor ofseti (+1.5 ön, -1.5 arka):
        # Bu ofset kalkışta arka motorların önceden havalanmasını ve öne fırlamayı sıfırlar!
        bias_pitch = 1.5
        m0 = self.actual_throttle + u_z + bias_pitch + u_pitch + u_roll - u_yaw  # Sol Ön
        m1 = self.actual_throttle + u_z - bias_pitch - u_pitch + u_roll + u_yaw  # Sol Arka
        m2 = self.actual_throttle + u_z + bias_pitch + u_pitch - u_roll + u_yaw  # Sağ Ön
        m3 = self.actual_throttle + u_z - bias_pitch - u_pitch - u_roll - u_yaw  # Sağ Arka

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
                    node.actual_throttle = node.hover_throttle  # Yerden temiz ve direkt kalkış (yerde kaymadan)
                    node.target_pitch = node.pitch_trim
                    node.target_roll = 0.0
                    node.target_yaw_rate = 0.0
                    node.pitch_command_dir = 0.0
                    node.roll_command_dir = 0.0
                    node.yaw_command_dir = 0.0
                    node.climb_command_dir = 0.0
                    node.last_climb_time = 0.0
                    node.integral_pitch = 0.0
                    node.integral_roll = 0.0
                    node.altitude_hold_active = True
                    if node.has_odom and node.pos_z < 0.4:
                        # Yerden yumuşak ve salınımsız kalkış (~1 metreye süzülür ve sabitlenir)
                        node.climb_command_dir = 1.0
                        node.climb_fast = False
                        node.last_climb_time = time.time() + 0.85
                        status_msg = f"🎯 HOVER KALKIŞ ({node.hover_throttle:.1f} rad/s)"
                    else:
                        node.climb_command_dir = 0.0
                        node.last_climb_time = 0.0
                        status_msg = f"🎯 HOVER KİLİDİ ({node.hover_throttle:.1f} rad/s)"
                elif key in ['w', 'W']:
                    node.climb_command_dir = 1.0
                    node.climb_fast = (key == 'W')
                    node.last_climb_time = time.time()
                    node.altitude_hold_active = True
                    if node.target_throttle < 1150.0:
                        node.target_throttle = node.hover_throttle
                        node.actual_throttle = node.hover_throttle
                    status_msg = "🚀 HIZLI TIRMANIŞ" if key == 'W' else "⬆️ YÜKSELİYOR..."
                elif key in ['s', 'S']:
                    node.climb_command_dir = -1.0
                    node.climb_fast = (key == 'S')
                    node.last_climb_time = time.time()
                    node.altitude_hold_active = True
                    if node.target_throttle < 1150.0:
                        node.target_throttle = node.hover_throttle
                        node.actual_throttle = node.hover_throttle
                    status_msg = "⏬ HIZLI ALÇALIŞ" if key == 'S' else "⬇️ ALÇALIYOR..."
                elif key == ' ':
                    node.altitude_hold_active = False
                    node.target_throttle = 0.0
                    node.actual_throttle = 0.0
                    node.climb_command_dir = 0.0
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
                    status_msg = "Geri Süzül (Fren)"
                elif key in ['\x1b[D', 'j', 'J']:  # Left Arrow
                    node.roll_command_dir = -1.0
                    node.last_roll_time = time.time()
                    status_msg = "Sola Süzül"
                elif key in ['\x1b[C', 'l', 'L']:  # Right Arrow
                    node.roll_command_dir = 1.0
                    node.last_roll_time = time.time()
                    status_msg = "Sağa Süzül"
                elif key in ['9', '[']:
                    node.pitch_trim = min(math.radians(3.0), node.pitch_trim + math.radians(0.05))
                    status_msg = f"Trim Geri (P:{math.degrees(node.pitch_trim):+.2f}°)"
                elif key in ['0', ']']:
                    node.pitch_trim = max(-math.radians(3.0), node.pitch_trim - math.radians(0.05))
                    status_msg = f"Trim İleri (P:{math.degrees(node.pitch_trim):+.2f}°)"
                elif key in ['a', 'A']:
                    node.yaw_command_dir = 1.0
                    node.yaw_fast = (key == 'A')
                    node.last_yaw_time = time.time()
                    status_msg = "⚡ HIZLI Sola Dönüş" if key == 'A' else "Sola Dönüş (Yaw)"
                elif key in ['d', 'D']:
                    node.yaw_command_dir = -1.0
                    node.yaw_fast = (key == 'D')
                    node.last_yaw_time = time.time()
                    status_msg = "⚡ HIZLI Sağa Dönüş" if key == 'D' else "Sağa Dönüş (Yaw)"
                elif key in ['x', 'X']:
                    node.pitch_trim = 0.0  # Trimi de sıfırla
                    node.pitch_command_dir = 0.0
                    node.roll_command_dir = 0.0
                    node.yaw_command_dir = 0.0
                    node.climb_command_dir = 0.0
                    node.last_climb_time = 0.0
                    node.target_pitch = 0.0
                    node.target_roll = 0.0
                    node.target_yaw_rate = 0.0
                    node.integral_pitch = 0.0
                    node.integral_roll = 0.0
                    node.target_throttle = node.hover_throttle
                    node.actual_throttle = node.hover_throttle
                    node.altitude_hold_active = True
                    status_msg = f"Durdur & Hover ({node.hover_throttle:.1f})"
                elif key in ['p', 'P']:
                    node.pid_enabled = not node.pid_enabled
                    state = "AÇIK" if node.pid_enabled else "KAPALI"
                    status_msg = f"PID Dengeleyici: {state}"
                elif key in ['m', 'M']:
                    node.trigger_ground_mode()
                    status_msg = "🦿 Kollar KARA MODUNA alındı!"
                elif key in ['n', 'N']:
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
            trim_str = f"T:{math.degrees(node.pitch_trim):+.2f}°|" if abs(node.pitch_trim) > 1e-3 else ""
            vel_str = f"Vy:{node.vel_y:+4.1f}m/s|" if node.has_odom else ""
            alt_str = f"Z:{node.pos_z:4.2f}m(Vz:{node.vel_z:+4.1f})|" if node.has_odom else ""

            sys.stdout.write(
                f"\r[{pid_tag}|{imu_tag}] GAZ:{node.actual_throttle:4.0f}/{node.target_throttle:4.0f} | "
                f"{alt_str}{vel_str}Açı:{trim_str}P:{cur_p_deg:+4.1f}° R:{cur_r_deg:+4.1f}° | "
                f"M:[{m0:.0f}, {m1:.0f}, {m2:.0f}, {m3:.0f}] | {status_msg:<28}"
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
