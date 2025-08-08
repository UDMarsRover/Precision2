import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

SERVO_PIN = 12
# The pigpio library uses microseconds for pulse width
MIN_PULSE = 500
MAX_PULSE = 2500

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.pi = pigpio.pi()  # Connect to the local pigpiod daemon
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpiod daemon.")
            raise Exception("Pigpiod not connected")

        # Set pin mode to output
        self.pi.set_mode(SERVO_PIN, pigpio.OUTPUT)

        self.last_position = None
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")
        self.pi.set_servo_pulsewidth(SERVO_PIN, 1500) # 1500us is center

    def listener_callback(self, msg):
        # Scale ROS position (-1.0 to 1.0) to pigpio pulse width (500 to 2500)
        pulse_width = int(((msg.data + 1) / 2) * (MAX_PULSE - MIN_PULSE) + MIN_PULSE)
        
        if self.last_position is None or abs(pulse_width - self.last_position) > 10: # Check for significant change in us
            self.pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
            self.last_position = pulse_width
            self.get_logger().info(f"Set servo to pulse width: {pulse_width}us")

    def destroy_node(self):
        self.pi.set_servo_pulsewidth(SERVO_PIN, 0) # Turn off PWM
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()