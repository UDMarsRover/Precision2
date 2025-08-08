import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio

SERVO_PIN = 12
MIN_PULSE = 500
MAX_PULSE = 2500

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.h = lgpio.gpiochip_open(0)  # Open the first GPIO chip
        if self.h < 0:
            self.get_logger().error("Could not open GPIO chip.")
            raise Exception("GPIO chip not opened")

        lgpio.gpio_claim_output(self.h, SERVO_PIN)
        self.last_position = None
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")
        self.set_servo_pulsewidth(1500) # 1500us is center

    def set_servo_pulsewidth(self, pulse_width):
        # lgpio uses gpio_send_pulse to send a pulse, but for servos we use gpio_servo
        lgpio.gpio_servo(self.h, SERVO_PIN, pulse_width)

    def listener_callback(self, msg):
        pulse_width = int(((msg.data + 1) / 2) * (MAX_PULSE - MIN_PULSE) + MIN_PULSE)
        if self.last_position is None or abs(pulse_width - self.last_position) > 10:
            self.set_servo_pulsewidth(pulse_width)
            self.last_position = pulse_width
            self.get_logger().info(f"Set servo to pulse width: {pulse_width}us")

    def destroy_node(self):
        self.set_servo_pulsewidth(0) # Turn off PWM
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
