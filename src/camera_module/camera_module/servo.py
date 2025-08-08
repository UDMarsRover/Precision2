import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio

SERVO_PIN = 12
SERVO_FREQ = 50  # Standard servo frequency in Hz
MIN_PULSE = 500  # Minimum pulse width in microseconds
MAX_PULSE = 2500 # Maximum pulse width in microseconds

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.h = lgpio.gpiochip_open(0)  # Open the first GPIO chip
        if self.h < 0:
            self.get_logger().error("Could not open GPIO chip.")
            raise Exception("GPIO chip not opened")

        # Start PWM on the servo pin with an initial pulse width of 1500us (center)
        lgpio.tx_pwm(self.h, SERVO_PIN, SERVO_FREQ, 1500)
        self.last_position = None
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")

    def set_servo_pulsewidth(self, pulse_width):
        """Sets the servo pulse width using lgpio's PWM function."""
        # The lgpio.tx_pwm function sends a single PWM pulse
        lgpio.tx_pwm(self.h, SERVO_PIN, SERVO_FREQ, pulse_width)

    def listener_callback(self, msg):
        # Map the ROS position (-1.0 to 1.0) to pulse width (500 to 2500)
        pulse_width = int(((msg.data + 1) / 2) * (MAX_PULSE - MIN_PULSE) + MIN_PULSE)
        pulse_width = max(MIN_PULSE, min(MAX_PULSE, pulse_width))
        
        # Only update if position changes significantly to reduce calls
        if self.last_position is None or abs(pulse_width - self.last_position) > 10:
            self.set_servo_pulsewidth(pulse_width)
            self.last_position = pulse_width
            self.get_logger().info(f"Set servo to pulse width: {pulse_width}us")

    def destroy_node(self):
        # Stop PWM by setting the pulse width to 0
        lgpio.tx_pwm(self.h, SERVO_PIN, SERVO_FREQ, 0)
        # Release the GPIO chip
        lgpio.gpiochip_close(self.h)
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