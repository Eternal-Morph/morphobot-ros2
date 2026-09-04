#!/usr/bin/env python3
import sys
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class ModeControllerNode(Node):
    def __init__(self, requested_mode=None, transition_time=3.0):
        super().__init__('mode_controller_node')
        
        self.declare_parameter('mode', 'default_mode')
        self.declare_parameter('transition_time', transition_time)
        
        param_mode = self.get_parameter('mode').get_parameter_value().string_value
        param_time = self.get_parameter('transition_time').get_parameter_value().double_value
        
        if requested_mode:
            self.mode = requested_mode
        else:
            self.mode = param_mode
            
        self.transition_time = param_time if param_time > 0 else 3.0

        self.joints = [
            'sol_on_ground_mode', 'sol_on_air_mode',
            'sag_on_ground_mode', 'sag_on_air_mode',
            'sol_arka_ground_mode', 'sol_arka_air_mode',
            'sag_arka_ground_mode', 'sag_arka_air_mode'
        ]

        # Eklem açı hedefleri ( default_mode, ground_mode, air_mode )
        self.mode_targets = {
            'default_mode': {
                'sol_on_ground_mode': 0.0,
                'sol_on_air_mode': 0.0,
                'sag_on_ground_mode': 0.0,
                'sag_on_air_mode': 0.0,
                'sol_arka_ground_mode': 0.0,
                'sol_arka_air_mode': 0.0,
                'sag_arka_ground_mode': 0.0,
                'sag_arka_air_mode': 0.0
            },
            'ground_mode': {
                'sol_on_ground_mode': 1.5708,
                'sol_on_air_mode': 0.0,
                'sag_on_ground_mode': -1.5708,
                'sag_on_air_mode': 0.0,
                'sol_arka_ground_mode': -1.5708,
                'sol_arka_air_mode': 0.0,
                'sag_arka_ground_mode': 1.5708,
                'sag_arka_air_mode': 0.0
            },
            'air_mode': {
                'sol_on_ground_mode': 1.5708,
                'sol_on_air_mode': -1.5708,
                'sag_on_ground_mode': -1.5708,
                'sag_on_air_mode': 1.5708,
                'sol_arka_ground_mode': -1.5708,
                'sol_arka_air_mode': 1.5708,
                'sag_arka_ground_mode': 1.5708,
                'sag_arka_air_mode': -1.5708
            }
        }

        # Mod ismini normalize et
        mode_key = self.mode.lower()
        if mode_key in ['default', 'default_mode', 'varsayilan']:
            mode_key = 'default_mode'
        elif mode_key in ['ground', 'ground_mode', 'kara', 'kara_modu']:
            mode_key = 'ground_mode'
        elif mode_key in ['air', 'air_mode', 'hava', 'hava_modu']:
            mode_key = 'air_mode'
        else:
            self.get_logger().warn(f"Bilinmeyen mod: '{self.mode}'. Varsayılan olarak 'default_mode' seçiliyor.")
            mode_key = 'default_mode'

        end_targets = self.mode_targets[mode_key]

        # Gazebo konuları
        self.pubs = {}
        for j in self.joints:
            topic = f'/eternalmorph/{j}/cmd_pos'
            self.pubs[j] = self.create_publisher(Float64, topic, 10)

        # RViz konuları
        self.js_cmd_pub = self.create_publisher(JointState, '/joint_states_cmd', 10)
        self.js_direct_pub = self.create_publisher(JointState, '/joint_states', 10)

        # /joint_states konusunu dinle (mevcut pozisyonu almak için)
        self.current_joint_states = {}
        self.js_sub = self.create_subscription(JointState, '/joint_states', self._js_callback, 10)

        # Mevcut joint_state gelmesi için kısa bir süre spin yap
        start_wait = time.time()
        while time.time() - start_wait < 0.2:
            rclpy.spin_once(self, timeout_sec=0.05)

        # Başlangıç pozisyonlarını belirle
        start_targets = {}
        for j in self.joints:
            if j in self.current_joint_states:
                start_targets[j] = self.current_joint_states[j]
            else:
                # Joint state henüz alınamadıysa varsayılan 0.0
                start_targets[j] = 0.0

        self.get_logger().info(f"Mod geçişi başlatılıyor: '{mode_key}' (Süre: {self.transition_time}s)...")

        # Sinüzoidal yumuşatılmış geçiş (S-curve)
        start_time = time.time()
        while rclpy.ok():
            elapsed = time.time() - start_time
            progress = min(1.0, elapsed / self.transition_time)

            factor = 0.5 * (1.0 - math.cos(math.pi * progress))

            current_positions = {}
            for j in self.joints:
                s_val = start_targets[j]
                e_val = end_targets[j]
                current_positions[j] = s_val + (e_val - s_val) * factor

            # 1. Gazebo için Float64 komutları
            for j, val in current_positions.items():
                msg = Float64()
                msg.data = float(val)
                self.pubs[j].publish(msg)

            # 2. RViz için JointState mesajı
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = list(current_positions.keys())
            js_msg.position = [float(current_positions[j]) for j in js_msg.name]

            self.js_cmd_pub.publish(js_msg)
            self.js_direct_pub.publish(js_msg)

            if progress >= 1.0:
                break

            time.sleep(0.02)

        # Hedef konumda tutmak için kısa bir süre daha yayınla
        end_time = time.time()
        while time.time() - end_time < 0.5 and rclpy.ok():
            for j, val in end_targets.items():
                msg = Float64()
                msg.data = float(val)
                self.pubs[j].publish(msg)
            time.sleep(0.05)

        self.get_logger().info(f"'{mode_key}' pozisyonuna başarıyla geçildi!")

    def _js_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joints:
                self.current_joint_states[name] = pos

def parse_mode_from_args(argv):
    requested_mode = None
    for arg in argv[1:]:
        if arg.startswith('mode:='):
            requested_mode = arg.split(':=', 1)[1]
        elif arg.startswith('--ros-args') or arg.startswith('-r') or arg.startswith('__'):
            continue
        elif not arg.startswith('-'):
            requested_mode = arg
    return requested_mode

def main(args=None):
    rclpy.init(args=args)
    cli_mode = parse_mode_from_args(sys.argv)
    node = ModeControllerNode(requested_mode=cli_mode)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
