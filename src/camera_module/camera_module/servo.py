import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio

SERVO_PIN = 12
SERVO_FREQ = 50  # Standard servo frequency in Hz
MIN_PULSE_DUTY = 25  # Minimum pulse width in microseconds
MAX_PULSE_DUTY = 125 # Maximum pulse width in microseconds

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.h = lgpio.gpiochip_open(0)  # Open the first GPIO chip
        if self.h < 0:
            self.get_logger().error("Could not open GPIO chip.")
            raise Exception("GPIO chip not opened")

        # Set the frequency for the PWM on the pin
        lgpio.gpio_pwm_frequency(self.h, SERVO_PIN, SERVO_FREQ)
        # Set initial pulse width to center
        lgpio.gpio_pwm_dutycycle(self.h, SERVO_PIN, 75)

        self.last_position = None
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")

    def listener_callback(self, msg):
        # Map the ROS position (-1.0 to 1.0) to a PWM duty cycle (25 to 125)
        duty_cycle = int(((msg.data + 1) / 2) * (MAX_PULSE_DUTY - MIN_PULSE_DUTY) + MIN_PULSE_DUTY)
        
        # Only update if the position changes significantly
        if self.last_position is None or abs(duty_cycle - self.last_position) > 1:
            lgpio.gpio_pwm_dutycycle(self.h, SERVO_PIN, duty_cycle)
            self.last_position = duty_cycle
            self.get_logger().info(f"Set servo duty cycle: {duty_cycle}")

    def destroy_node(self):
        # Stop PWM by setting the duty cycle to 0
        lgpio.gpio_pwm_dutycycle(self.h, SERVO_PIN, 0)
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