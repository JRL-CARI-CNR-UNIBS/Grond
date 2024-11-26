import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoyToVelocityController(Node):
    def __init__(self):
        super().__init__('joy_to_velocity_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'forward_velocity_controller/commands', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self._joy_msg = None
        self.timer_period = 0.1  # seconds (set your desired rate here)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def joy_callback(self, msg):
        self._joy_msg = msg

    def timer_callback(self):
        if self._joy_msg != None:
            msg = self._joy_msg
            for_dir = msg.axes[1]
            side_dir = msg.axes[0]
            msg = Float64MultiArray()
            vel_value = 0.6
            msg.data = [vel_value] * 4

            if for_dir == 0 and side_dir == 0:
                msg.data = [0.0,0.0,0.0,0.0] 
            else:
                # if for_dir != 0:
                #     print(for_dir)
                #     msg.data = [for_dir * vel_value] * 4
                # elif side_dir != 0:
                #     print(side_dir)
                msg.data = [(for_dir + side_dir) * vel_value,
                            (for_dir - side_dir) * vel_value,
                            (for_dir - side_dir) * vel_value,
                            (for_dir + side_dir) * vel_value] 


            self.publisher_.publish(msg)


        # axes = msg.axes
        # if len(axes) >= 2:
        #     velocity_msg = Float64MultiArray()
        #     velocity_msg.data = [axes[0], axes[1]]
        #     self.publisher_.publish(velocity_msg)



def main(args=None):
    rclpy.init(args=args)
    joy_to_velocity_controller = JoyToVelocityController()
    rclpy.spin(joy_to_velocity_controller)
    joy_to_velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()