import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from collections import deque
from control import DynamixelMX

# Constants for motor and smoothing
# Replace with your specific motor settings
SERIAL_PORT = '/dev/ttyAMA0'
MOTOR_ID = 1
BAUDRATE = 57600
SMOOTHING_WINDOW_SIZE = 5
TRUE_CENTER = 1830

class DynamixelMotorNode(Node):
    """
    A ROS 2 node that controls a Dynamixel motor using a smoothed Float32 topic.
    """
    def __init__(self):
        super().__init__('dynamixel_motor_node')

        self.get_logger().info("Initializing Dynamixel motor node...")
        
        # Initialize motor
        try:
            self.motor = DynamixelMX(SERIAL_PORT, MOTOR_ID, BAUDRATE)
            self.min_pos = self.motor.get_min_position()
            self.max_pos = self.motor.get_max_position()
            self.pos_delta = ((self.min_pos + self.max_pos) / 2) - TRUE_CENTER
            self.get_logger().info(f"Connected to Dynamixel motor. Position range: {self.min_pos} to {self.max_pos}.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Dynamixel motor: {e}")
            raise Exception("Motor initialization failed")

        # Set up the moving average filter
        self.position_history = deque(maxlen=SMOOTHING_WINDOW_SIZE)

        # Create a subscriber to the 'motor_position' topic
        self.subscription = self.create_subscription(
            Float32,
            'camera_yaw',
            self.listener_callback,
            10
        )
        self.get_logger().info("Dynamixel motor node is ready. Waiting for messages on 'camera_yaw' topic.")

    def listener_callback(self, msg):
        """
        Callback function for the subscriber.
        It processes incoming Float32 messages to control the motor.
        """
        # Add the new data to the smoothing window
        self.position_history.append(msg.data)

        # Calculate the average of the values in the window for smoothing
        smoothed_data = sum(self.position_history) / len(self.position_history)

        # Clamp the smoothed data to the valid range of the topic
        clamped_data = max(-1.0, min(1.0, smoothed_data))

        # Map the clamped data from the [-1.0, 1.0] range to the motor's position range
        # Formula: new_value = ((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        mapped_position = ((clamped_data - (-1.0)) / (1.0 - (-1.0))) * (self.max_pos - self.min_pos) + self.min_pos
        
        # Write the new goal position to the motor
        # Dynamixel positions are typically integers, so we convert the float to int
        mapped_position = mapped_position - self.pos_delta
        if mapped_position < self.min_pos:
            mapped_position = self.min_pos
        self.motor.write_goal_position(int(mapped_position))
        self.get_logger().info(f"Received: {msg.data:.2f}, Smoothed: {clamped_data:.2f}, Set position to: {int(mapped_position)}")

    def destroy_node(self):
        """
        Properly shuts down the motor and the node.
        """
        self.motor.write_torque_enable(0) # Disable torque to free the motor
        self.get_logger().info("Torque disabled. Node shutting down.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt detected, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
