import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import curses

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.key_values = {'a': 0, 'z': 0, 's': 0, 'x': 0, 'd': 0, 'c': 0, 'f': 0, 'v': 0}

    def publish_key_presses(self, key = None):
        self.key_values = {'a': 0, 'z': 0, 's': 0, 'x': 0, 'd': 0, 'c': 0, 'f': 0, 'v': 0}
        vel_value = 0.4
        if key and (key in self.key_values or key == 'j' or key == 'm'):
            if key in 'azsxdcfv':
                self.key_values[key] = vel_value
            elif key in 'j':
                self.key_values['a'] = vel_value
                self.key_values['s'] = vel_value
                self.key_values['d'] = vel_value
                self.key_values['f'] = vel_value
            elif key in 'm':
                self.key_values['z'] = vel_value
                self.key_values['x'] = vel_value
                self.key_values['c'] = vel_value
                self.key_values['v'] = vel_value
            else:
                self.key_values[key] = 0.0

        msg = Float64MultiArray()
        msg.data = [self.key_values['a'] - self.key_values['z'],
                    self.key_values['s'] - self.key_values['x'],
                    self.key_values['d'] - self.key_values['c'],
                    self.key_values['f'] - self.key_values['v']]

        self.publisher_.publish(msg)
            # self.get_logger().info('Publishing: %s' % str(msg.data))

def main(args=None):
    rclpy.init(args=args)

    node = KeyPublisher()
    rate = node.create_rate(5)  # 10Hz

    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)  # Make getch non-blocking

    stdscr.addstr("Press 'a/z', 's/x', 'd/c', 'f/v' to increment/decrement values. Press 'q' to quit.\n")

    while rclpy.ok():
        rclpy.spin_once(node)
        c = stdscr.getch()
        old_c = c
        while c != -1:  # Read all available characters
            old_c = c
            c = stdscr.getch()
        if old_c == ord('q'):
            break
        elif old_c != -1 and chr(old_c) in 'azsxdcfvjm':
            node.publish_key_presses(chr(old_c))
        else:
            node.publish_key_presses()
        rate.sleep()

    curses.endwin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()