import tkinter as tk  # Import tkinter library for GUI functionality
import rclpy  # Import rclpy for ROS2 functionality
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Import Twist message for /cmd_vel topic


class DirectionalPad(Node):
    def __init__(self):
        # Initialize the ROS2 node
        rclpy.init(args=None)  # Initialize rclpy
        super().__init__('directional_pad_node')  # Initialize as a ROS2 node with a specified name

        # Publisher for /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # Create a publisher with a queue size of 10

        self.max_lin_vel = 0.1
        self.max_ang_vel = 0.1

        # State variables for button presses
        self.button_state = {
            "Up": False,
            "Down": False,
            "Left": False,
            "Right": False,
            "L": False,
            "R": False
        }

        # Set up the main application window
        self.root = tk.Tk()  # Initialize the main tkinter window
        self.root.title("Directional Pad")  # Set the window title
        self.root.geometry("200x200")  # Set the window size

        # Create buttons
        self.create_buttons()

        # Start periodic checking for button presses
        self.check_buttons()  # Begin checking for button presses at regular intervals

        # Start the tkinter main event loop
        self.root.mainloop()  # Run the tkinter event loop

    def create_buttons(self):
        """Creates and places buttons in the application window."""
        # Create and place buttons in a grid layout
        self.btn_up = tk.Button(self.root, text="↑")
        self.btn_down = tk.Button(self.root, text="↓")
        self.btn_left = tk.Button(self.root, text="←")
        self.btn_right = tk.Button(self.root, text="→")
        self.btn_a = tk.Button(self.root, text="R")  # New button "A"
        self.btn_b = tk.Button(self.root, text="L")  # New button "B"

        # Assign press and release bindings to each button
        self.btn_up.bind("<ButtonPress>", lambda event: self.on_button_press("Up"))
        self.btn_up.bind("<ButtonRelease>", lambda event: self.on_button_release("Up"))

        self.btn_down.bind("<ButtonPress>", lambda event: self.on_button_press("Down"))
        self.btn_down.bind("<ButtonRelease>", lambda event: self.on_button_release("Down"))

        self.btn_left.bind("<ButtonPress>", lambda event: self.on_button_press("Left"))
        self.btn_left.bind("<ButtonRelease>", lambda event: self.on_button_release("Left"))

        self.btn_right.bind("<ButtonPress>", lambda event: self.on_button_press("Right"))
        self.btn_right.bind("<ButtonRelease>", lambda event: self.on_button_release("Right"))

        self.btn_a.bind("<ButtonPress>", lambda event: self.on_button_press("R"))
        self.btn_a.bind("<ButtonRelease>", lambda event: self.on_button_release("R"))

        self.btn_b.bind("<ButtonPress>", lambda event: self.on_button_press("L"))
        self.btn_b.bind("<ButtonRelease>", lambda event: self.on_button_release("L"))

        # Place buttons in a grid layout
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)
        self.btn_a.grid(row=0, column=2, padx=5, pady=5)  # Place "A" button above "Right"
        self.btn_b.grid(row=0, column=0, padx=5, pady=5)  # Place "B" button above "Left"

    def on_button_press(self, direction):
        """Handles button press event."""
        self.button_state[direction] = True  # Set the button's state to pressed

    def on_button_release(self, direction):
        """Handles button release event."""
        self.button_state[direction] = False  # Set the button's state to not pressed

    def check_buttons(self):
        """Periodically checks the state of each button."""
        if any(self.button_state.values()):
            # If any button is pressed, publish the corresponding command
            for direction, pressed in self.button_state.items():
                if pressed:
                    self.publish_cmd_vel(direction)
        else:
            # If no button is pressed, publish an empty (zeroed) Twist message
            self.publish_empty_twist()

        self.root.after(100, self.check_buttons)  # Schedule this function to repeat every 100 ms

    def publish_cmd_vel(self, direction):
        """Publishes a Twist message to /cmd_vel based on the button direction."""
        twist_msg = Twist()

        # Set linear or angular velocity based on direction
        if direction == "Up":
            twist_msg.linear.x = self.max_lin_vel  # Move forward
        elif direction == "Down":
            twist_msg.linear.x = -self.max_lin_vel  # Move backward
        elif direction == "Left":
            twist_msg.linear.y = self.max_lin_vel  # Move left
        elif direction == "Right":
            twist_msg.linear.y = -self.max_lin_vel  # Move right
        elif direction == "L":
            twist_msg.angular.z = self.max_ang_vel  # Turn left
        elif direction == "R":
            twist_msg.angular.z = -self.max_ang_vel  # Turn right

        self.publisher_.publish(twist_msg)  # Publish the Twist message to /cmd_vel
        self.get_logger().info(f"Publishing {direction} command to /cmd_vel")

    def publish_empty_twist(self):
        """Publishes an empty Twist message (all velocities set to zero)."""
        twist_msg = Twist()  # By default, all values in Twist are zero
        self.publisher_.publish(twist_msg)
        self.get_logger().info("Publishing empty command to /cmd_vel")

    def shutdown(self):
        """Shutdown the ROS2 node gracefully."""
        rclpy.shutdown()  # Shutdown rclpy when done


# Instantiate and run the DirectionalPad application if executed directly
if __name__ == "__main__":
    try:
        dpad_app = DirectionalPad()
    except KeyboardInterrupt:
        print("Shutting down Directional Pad application")
        dpad_app.shutdown()
