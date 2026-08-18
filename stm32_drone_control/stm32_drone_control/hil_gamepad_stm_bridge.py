#!/usr/bin/env python3
# ==============================================================================
# HIL Simülasyon Köprüsü - STM32 <---> ROS 2 / Gazebo & Gamepad
# ==============================================================================

import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import math
import struct
import time
import serial
import pygame

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators


class HilBridgeNode(Node):
    def __init__(self):
        super().__init__('hil_bridge_node')

        # 1. Parametreler
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_rotor_velocity', 1000.0)  # rad/s
        self.declare_parameter('imu_topic', '/imu/sim_data')
        self.declare_parameter('motor_topic', '/stm32_drone/command/motor_speed')

        # IMU Birim Ayarları
        self.declare_parameter('gyro_in_rad_s', False)
        self.declare_parameter('accel_in_mps2', False)

        # IMU İşaret ve Eksen Ayarları
        self.declare_parameter('invert_gyro_x', True)
        self.declare_parameter('invert_gyro_y', False)
        self.declare_parameter('invert_gyro_z', False)
        self.declare_parameter('invert_accel_x', False)
        self.declare_parameter('invert_accel_y', False)

        # Motor Eşleme (STM32 [M1, M2, M3, M4] -> Gazebo [Rotor0(FR), Rotor1(RL), Rotor2(FL), Rotor3(RR)])
        self.declare_parameter('motor_order', '0,1,2,3')

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.max_rotor_velocity = self.get_parameter('max_rotor_velocity').get_parameter_value().double_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.motor_topic = self.get_parameter('motor_topic').get_parameter_value().string_value

        self.gyro_in_rad_s = self.get_parameter('gyro_in_rad_s').get_parameter_value().bool_value
        self.accel_in_mps2 = self.get_parameter('accel_in_mps2').get_parameter_value().bool_value

        self.invert_gyro_x = self.get_parameter('invert_gyro_x').get_parameter_value().bool_value
        self.invert_gyro_y = self.get_parameter('invert_gyro_y').get_parameter_value().bool_value
        self.invert_gyro_z = self.get_parameter('invert_gyro_z').get_parameter_value().bool_value
        self.invert_accel_x = self.get_parameter('invert_accel_x').get_parameter_value().bool_value
        self.invert_accel_y = self.get_parameter('invert_accel_y').get_parameter_value().bool_value

        order_str = self.get_parameter('motor_order').get_parameter_value().string_value
        try:
            self.motor_indices = [int(x.strip()) for x in order_str.split(',')]
            if len(self.motor_indices) != 4:
                self.motor_indices = [0, 1, 2, 3]
        except Exception:
            self.motor_indices = [0, 1, 2, 3]

        self.pwm_min = 1000.0
        self.pwm_max = 2000.0

        # 2. Sensör ve Kontrol Değişkenleri
        self.sim_accel_x, self.sim_accel_y, self.sim_accel_z = 0.0, 0.0, 1.0
        self.sim_gyro_x,  self.sim_gyro_y,  self.sim_gyro_z  = 0.0, 0.0, 0.0

        self.filtered_throttle = 1000.0
        self.rx_buffer = bytearray()

        # Telemetri değişkenleri
        self.m1, self.m2, self.m3, self.m4 = 1000, 1000, 1000, 1000
        self.est_r, self.est_p, self.est_y = 0.0, 0.0, 0.0
        self.st_m = 0

        # 3. Gamepad Başlatma
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        if pygame.joystick.get_count() == 0:
            self.get_logger().warn("⚠️ Gamepad bulunamadı! Lütfen bir Gamepad/Joystick bağlayın.")
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"✅ Gamepad Bağlandı: {self.joystick.get_name()}")

        # 4. Seri Port Başlatma
        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.002)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"✅ Seri Port Açıldı: {self.serial_port_name} @ {self.baud_rate}")
        except Exception as e:
            self.get_logger().error(f"❌ Seri Port Hatası ({self.serial_port_name}): {e}")

        # 5. ROS 2 QoS ve Topic Bağlantıları
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )

        self.motor_pub = self.create_publisher(
            Actuators,
            self.motor_topic,
            1
        )

        # 6. HIL Ana Döngü Zamanlayıcısı (50 Hz -> 20 ms)
        self.timer = self.create_timer(0.02, self.hil_loop_callback)
        self.get_logger().info(f"🚀 ROS 2 HIL Köprüsü Çalışıyor (50 Hz)... Motor Sırası: {self.motor_indices}")

    def imu_callback(self, msg: Imu):
        # Accel Dönüşümü
        if self.accel_in_mps2:
            acc_x = msg.linear_acceleration.x
            acc_y = msg.linear_acceleration.y
            acc_z = msg.linear_acceleration.z
        else:
            acc_x = msg.linear_acceleration.x / 9.80665
            acc_y = msg.linear_acceleration.y / 9.80665
            acc_z = msg.linear_acceleration.z / 9.80665

        # Gyro Dönüşümü
        if self.gyro_in_rad_s:
            gyro_x = msg.angular_velocity.x
            gyro_y = msg.angular_velocity.y
            gyro_z = msg.angular_velocity.z
        else:
            rad_to_deg = 180.0 / math.pi
            gyro_x = msg.angular_velocity.x * rad_to_deg
            gyro_y = msg.angular_velocity.y * rad_to_deg
            gyro_z = msg.angular_velocity.z * rad_to_deg

        self.sim_accel_x = -acc_x if self.invert_accel_x else acc_x
        self.sim_accel_y = -acc_y if self.invert_accel_y else acc_y
        self.sim_accel_z = acc_z

        self.sim_gyro_x = -gyro_x if self.invert_gyro_x else gyro_x
        self.sim_gyro_y = -gyro_y if self.invert_gyro_y else gyro_y
        self.sim_gyro_z = -gyro_z if self.invert_gyro_z else gyro_z

    def apply_smooth_expo(self, val, deadband=0.08, expo=0.70):
        if abs(val) < deadband:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        scaled = (abs(val) - deadband) / (1.0 - deadband)
        return sign * (expo * (scaled ** 3) + (1.0 - expo) * scaled)

    def pwm_to_rads(self, pwm_val):
        clamped = max(self.pwm_min, min(self.pwm_max, float(pwm_val)))
        ratio = (clamped - self.pwm_min) / (self.pwm_max - self.pwm_min)
        return ratio * self.max_rotor_velocity

    def hil_loop_callback(self):
        if self.joystick is None:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"✅ Gamepad Bağlandı: {self.joystick.get_name()}")

        raw_yaw, raw_thr, raw_roll, raw_pitch = 0.0, -1.0, 0.0, 0.0
        is_armed = 0
        if self.joystick is not None:
            for _ in pygame.event.get():
                pass

            raw_yaw = self.joystick.get_axis(0)
            raw_thr = -self.joystick.get_axis(1)

            num_axes = self.joystick.get_numaxes()
            if num_axes >= 5:
                raw_roll = self.joystick.get_axis(2) if abs(self.joystick.get_axis(2)) < 0.99 else self.joystick.get_axis(3)
                raw_pitch = -self.joystick.get_axis(3) if abs(self.joystick.get_axis(3)) < 0.99 else -self.joystick.get_axis(4)
            elif num_axes >= 4:
                raw_roll = self.joystick.get_axis(2)
                raw_pitch = -self.joystick.get_axis(3)

        if raw_thr > 0.05:
            thr_norm = (raw_thr - 0.05) / 0.95
            target_throttle = 1000.0 + (thr_norm ** 1.8) * 800.0
            is_armed = 1  # Gaz verilince ARM et
        else:
            target_throttle = 1000.0
            is_armed = 0  # Gaz sıfırken DISARM et (Yerde PID Integral Windup birikmesini önle!)

        self.filtered_throttle += (target_throttle - self.filtered_throttle) * 0.15
        throttle = int(self.filtered_throttle)

        target_roll  = round(self.apply_smooth_expo(raw_roll) * 25.0, 1)
        target_pitch = round(self.apply_smooth_expo(raw_pitch) * 25.0, 1)
        target_yaw   = round(self.apply_smooth_expo(raw_yaw) * 100.0, 1)

        robot_mode = 0  # MODE_AIR

        # 1. STM32'ye Gönder
        payload = struct.pack('<ffffffHfffBB',
                              self.sim_accel_x, self.sim_accel_y, self.sim_accel_z,
                              self.sim_gyro_x,  self.sim_gyro_y,  self.sim_gyro_z,
                              throttle, target_roll, target_pitch, target_yaw,
                              is_armed, robot_mode)

        checksum = sum(payload) & 0xFF
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'\xAA\x55' + payload + bytes([checksum]))
            except Exception as e:
                self.get_logger().error(f"Seri yazma hatası: {e}")

        # 2. STM32'den Oku
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))

                while len(self.rx_buffer) >= 29:
                    if self.rx_buffer[0] == 0x55 and self.rx_buffer[1] == 0xAA:
                        packet = self.rx_buffer[:29]
                        payload_rx = packet[2:28]
                        pkt_checksum = packet[28]

                        if (sum(payload_rx) & 0xFF) == pkt_checksum:
                            self.m1, self.m2, self.m3, self.m4, l_w, r_w, self.st_m, fail_r, self.est_r, self.est_p, self.est_y = struct.unpack('<HHHHHHBBfff', payload_rx)
                            self.rx_buffer = self.rx_buffer[29:]

                            # Motor Eşleme ve Gazebo'ya İletme
                            raw_pwms = [self.m1, self.m2, self.m3, self.m4]
                            mapped_pwms = [raw_pwms[idx] for idx in self.motor_indices]

                            motor_msg = Actuators()
                            motor_msg.header.stamp = self.get_clock().now().to_msg()
                            motor_msg.velocity = [
                                float(self.pwm_to_rads(mapped_pwms[0])),
                                float(self.pwm_to_rads(mapped_pwms[1])),
                                float(self.pwm_to_rads(mapped_pwms[2])),
                                float(self.pwm_to_rads(mapped_pwms[3]))
                            ]
                            self.motor_pub.publish(motor_msg)
                        else:
                            self.rx_buffer.pop(0)
                    else:
                        self.rx_buffer.pop(0)
            except Exception as e:
                self.get_logger().error(f"Seri okuma hatası: {e}")

        # Canlı Durum Satırı
        print(f"\r[ARM:{is_armed}] GAZ:{throttle:4d} | M1:{self.m1:4d} M2:{self.m2:4d} M3:{self.m3:4d} M4:{self.m4:4d} | R:{self.est_r:+5.1f}° P:{self.est_p:+5.1f}° | GyroX:{self.sim_gyro_x:+.1f} ", end="", flush=True)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HilBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("\nKöprü kapatıldı.")


if __name__ == '__main__':
    main()
