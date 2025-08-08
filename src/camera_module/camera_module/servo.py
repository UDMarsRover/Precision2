import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pigpio

SERVO_PIN = 12
MIN_PULSE = 500   # microseconds
MAX_PULSE = 2500  # microseconds

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.pi = pigpio.pi()  # Connect to local pigpiod daemon
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpiod daemon.")
            raise Exception("pigpio daemon not connected")

        # Set initial pulse width (center position)
        self.pi.set_servo_pulsewidth(SERVO_PIN, 1500)
        self.last_position = None
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")

    def set_servo_pulsewidth(self, pulse_width):
        self.pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

    def listener_callback(self, msg):
        pulse_width = int(((msg.data + 1) / 2) * (MAX_PULSE - MIN_PULSE) + MIN_PULSE)
        pulse_width = max(MIN_PULSE, min(MAX_PULSE, pulse_width))
        if self.last_position is None or abs(pulse_width - self.last_position) > 10:
            self.set_servo_pulsewidth(pulse_width)
            self.last_position = pulse_width
            self.get_logger().info(f"Set servo to pulse width: {pulse_width}us")

    def destroy_node(self):
        self.pi.set_servo_pulsewidth(SERVO_PIN, 0)
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
