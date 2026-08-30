#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

class PidTunerNode(Node):
    """
    ROS 2 Parametre Arayüzü ile STM32'ye canlı PID parametreleri yollayan Düğüm.
    Kullanıcı: 'ros2 param set /pid_tuner_node kp_roll 1.5' komutuyla anlık güncelleme yapabilir.
    """
    def __init__(self):
        super().__init__('pid_tuner_node')

        # Varsayılan PID parametrelerini tanımla
        self.declare_parameter('kp_roll', 1.0)
        self.declare_parameter('ki_roll', 0.0)
        self.declare_parameter('kd_roll', 0.05)

        self.declare_parameter('kp_pitch', 1.0)
        self.declare_parameter('ki_pitch', 0.0)
        self.declare_parameter('kd_pitch', 0.05)

        self.declare_parameter('setpoint_roll', 0.0)
        self.declare_parameter('setpoint_pitch', 0.0)

        # PID komut yayıncısı (/pid/tune_cmd)
        self.cmd_pub = self.create_publisher(String, '/pid/tune_cmd', 10)

        # Parametre değişim dinleyicisi
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("PID Tuner Düğümü Başlatıldı!")
        self.get_logger().info("Parametre değiştirmek için: ros2 param set /pid_tuner_node kp_roll 1.25")

    def parameters_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parametre Güncellendi: {param.name} = {param.value}")
            cmd_str = f"SET:{param.name.upper()}={param.value:.4f}\n"

            msg = String()
            msg.data = cmd_str
            self.cmd_pub.publish(msg)

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = PidTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
