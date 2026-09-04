#!/usr/bin/env python3
import os
import sys
import time
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

STATE_FILE = '/tmp/eternalmorph_state.json'

class ModeControllerNode(Node):
    def __init__(self, requested_mode=None, transition_time=3.0):
        super().__init__(
            'mode_controller_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        
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

        # DiffDrive tekerlek durdurma konusu
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # /joint_states konusunu dinle (mevcut pozisyonu almak için)
        self.current_joint_states = {}
        self.js_sub = self.create_subscription(JointState, '/joint_states', self._js_callback, 10)

        # Mevcut eklem açılarını oku (DDS keşfi için bekle)
        self.get_logger().info("Mevcut eklem durumları (/joint_states) okunuyor...")
        start_wait = time.time()
        while time.time() - start_wait < 2.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            # Tüm hedef eklemlerimizin açısı geldi mi kontrol et
            if all(j in self.current_joint_states for j in self.joints):
                break

        # Başlangıç pozisyonlarını belirle
        start_targets = {}
        self.gazebo_active = all(j in self.current_joint_states for j in self.joints)
        received_all = self.gazebo_active

        if received_all:
            for j in self.joints:
                start_targets[j] = self.current_joint_states[j]
            self.get_logger().info("Mevcut eklem pozisyonları Gazebo'dan başarıyla okundu.")
        else:
            # Fallback: Önbellekteki son bilinen modu oku
            cached_mode = self._read_cached_mode()
            self.get_logger().warn(f"Joint states zaman aşımına uğradı! Önbellekteki mod kullanılıyor: '{cached_mode}'")
            cached_targets = self.mode_targets.get(cached_mode, self.mode_targets['default_mode'])
            for j in self.joints:
                if j in self.current_joint_states:
                    start_targets[j] = self.current_joint_states[j]
                else:
                    start_targets[j] = cached_targets.get(j, 0.0)

        # Mevcut robot modunu tespit et
        current_mode = self._detect_current_mode(start_targets)
        self.get_logger().info(f"Tespit edilen başlangıç durumu: '{current_mode}'")

        # Geçiş aşamalarını belirle (air_mode geçişleri zorunlu olarak ground_mode üzerinden yapılır)
        stages = self._determine_mode_stages(current_mode, mode_key)
        self.get_logger().info(f"Planlanan geçiş rotası: {' -> '.join(stages)} (Hedef: {mode_key})")

        # Aşamaları sırayla icra et
        active_start_targets = dict(start_targets)
        for idx, stage_mode in enumerate(stages, 1):
            stage_targets = self.mode_targets[stage_mode]
            self.get_logger().info(f"--- Aşama {idx}/{len(stages)}: '{stage_mode}' pozisyonuna geçiliyor ({self.transition_time}s) ---")
            active_start_targets = self._execute_transition(active_start_targets, stage_targets, stage_mode, self.transition_time)

        # Başarıyla geçilen nihai modu önbelleğe kaydet
        self._save_cached_mode(mode_key)

        # DiffDrive tekerlek komutlarını sıfırla (tekerleklerin kaymasını/dönmesini durdur)
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        self.get_logger().info(f"Tüm aşamalar başarıyla tamamlandı! Robot '{mode_key}' konumunda.")

    def _detect_current_mode(self, current_positions):
        air_angle = abs(current_positions.get('sol_on_air_mode', 0.0))
        ground_angle = abs(current_positions.get('sol_on_ground_mode', 0.0))

        if air_angle > 0.6:
            return 'air_mode'
        elif ground_angle > 0.6:
            return 'ground_mode'
        elif ground_angle < 0.4 and air_angle < 0.4:
            return 'default_mode'
        return self._read_cached_mode()

    def _determine_mode_stages(self, current_mode, target_mode):
        if target_mode == current_mode:
            return [target_mode]

        # 1. air_mode'a gidiliyorsa ve robot ground_mode'da değilse: önce ground_mode'a geç
        if target_mode == 'air_mode':
            if current_mode != 'ground_mode':
                return ['ground_mode', 'air_mode']
            else:
                return ['air_mode']

        # 2. air_mode'dan başka bir moda gidiliyorsa: önce ground_mode'a geç
        if current_mode == 'air_mode':
            if target_mode != 'ground_mode':
                return ['ground_mode', target_mode]
            else:
                return ['ground_mode']

        # Standart geçiş (default <-> ground)
        return [target_mode]

    def _execute_transition(self, start_targets, end_targets, stage_mode, duration):
        start_time = time.time()
        while rclpy.ok():
            elapsed = time.time() - start_time
            progress = min(1.0, elapsed / duration)

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
            if not self.gazebo_active:
                self.js_direct_pub.publish(js_msg)

            if progress >= 1.0:
                break

            time.sleep(0.02)

        # Hedef konumda tutmak için kısa bir süre daha yayınla
        end_time = time.time()
        while time.time() - end_time < 0.3 and rclpy.ok():
            for j, val in end_targets.items():
                msg = Float64()
                msg.data = float(val)
                self.pubs[j].publish(msg)
            time.sleep(0.05)

        self._save_cached_mode(stage_mode)
        return end_targets

    def _js_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joints:
                self.current_joint_states[name] = pos

    def _read_cached_mode(self):
        try:
            if os.path.exists(STATE_FILE):
                with open(STATE_FILE, 'r') as f:
                    data = json.load(f)
                    return data.get('mode', 'default_mode')
        except Exception:
            pass
        return 'default_mode'

    def _save_cached_mode(self, mode_name):
        try:
            with open(STATE_FILE, 'w') as f:
                json.dump({'mode': mode_name, 'timestamp': time.time()}, f)
        except Exception:
            pass

def parse_args(argv):
    requested_mode = None
    transition_time = 3.0
    
    positional = []
    for arg in argv[1:]:
        if arg.startswith('mode:='):
            requested_mode = arg.split(':=', 1)[1]
        elif arg.startswith('transition_time:='):
            try:
                transition_time = float(arg.split(':=', 1)[1])
            except ValueError:
                pass
        elif arg.startswith('--ros-args') or arg.startswith('-r') or arg.startswith('__'):
            continue
        elif not arg.startswith('-'):
            positional.append(arg)

    if not requested_mode and len(positional) > 0:
        requested_mode = positional[0]
    if len(positional) > 1:
        try:
            transition_time = float(positional[1])
        except ValueError:
            pass

    return requested_mode, transition_time

def main(args=None):
    rclpy.init(args=args)
    cli_mode, cli_time = parse_args(sys.argv)
    node = ModeControllerNode(requested_mode=cli_mode, transition_time=cli_time)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
