import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 1)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(0.0 * math.sin(self.i)) ,float(0.0 * math.sin(self.i)) ,float(0.0 * math.sin(self.i)) ,float(0.5 * math.sin(self.i))]
        self.i = self.i + 0.1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()