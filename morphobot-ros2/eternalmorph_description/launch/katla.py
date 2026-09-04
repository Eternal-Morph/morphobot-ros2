#!/usr/bin/env python3
import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class ArmFolderNode(Node):
    def __init__(self, mode, fold_time=3.0):
        super().__init__(
            'arm_folder_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.mode = mode
        self.fold_time = fold_time

        joints = [
            'sol_on_ground_mode', 'sol_on_air_mode',
            'sag_on_ground_mode', 'sag_on_air_mode',
            'sol_arka_ground_mode', 'sol_arka_air_mode',
            'sag_arka_ground_mode', 'sag_arka_air_mode'
        ]

        self.pubs = {}
        for j in joints:
            topic = f'/eternalmorph/{j}/cmd_pos'
            self.pubs[j] = self.create_publisher(Float64, topic, 10)

        # RViz / JointStatePublisher konuları
        self.js_cmd_pub = self.create_publisher(JointState, '/joint_states_cmd', 10)
        self.js_direct_pub = self.create_publisher(JointState, '/joint_states', 10)

        if self.mode == 'katla':
            self.get_logger().info(f'Kollar {self.fold_time} saniyede yumuşakça katlanıyor...')
            start_targets = {j: 0.0 for j in joints}
            end_targets = {
                'sol_on_ground_mode': 1.571,
                'sol_on_air_mode': -1.571,
                'sag_on_ground_mode': -1.571,
                'sag_on_air_mode': 1.571,
                'sol_arka_ground_mode': -1.571,
                'sol_arka_air_mode': 1.571,
                'sag_arka_ground_mode': 1.571,
                'sag_arka_air_mode': -1.571
            }
        else:
            self.get_logger().info(f'Kollar {self.fold_time} saniyede yumuşakça açılıyor (Kara Modu)...')
            start_targets = {
                'sol_on_ground_mode': 1.571,
                'sol_on_air_mode': -1.571,
                'sag_on_ground_mode': -1.571,
                'sag_on_air_mode': 1.571,
                'sol_arka_ground_mode': -1.571,
                'sol_arka_air_mode': 1.571,
                'sag_arka_ground_mode': 1.571,
                'sag_arka_air_mode': -1.571
            }
            end_targets = {j: 0.0 for j in joints}

        start_time = time.time()
        while True:
            elapsed = time.time() - start_time
            progress = min(1.0, elapsed / self.fold_time)

            # Sinüzoidal yumuşatılmış ivmelenme/yavaşlama (S-curve)
            factor = 0.5 * (1.0 - math.cos(math.pi * progress))

            current_positions = {}
            for j in joints:
                s_val = start_targets[j]
                e_val = end_targets[j]
                current_positions[j] = s_val + (e_val - s_val) * factor

            # 1. Gazebo için Float64 pozisyon komutları
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

        # Son konumda tutmak için kısa bir süre daha yayınla
        end_time = time.time()
        while time.time() - end_time < 0.5:
            for j, val in end_targets.items():
                msg = Float64()
                msg.data = float(val)
                self.pubs[j].publish(msg)
            time.sleep(0.05)

        self.get_logger().info('Katlama komutu başarıyla tamamlandı!')

def main():
    mode = 'katla'
    fold_time = 3.0

    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
    if len(sys.argv) > 2:
        try:
            fold_time = float(sys.argv[2])
        except ValueError:
            pass

    rclpy.init()
    node = ArmFolderNode(mode, fold_time)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
