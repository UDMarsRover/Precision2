import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

SERVO_PIN = 12

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        # Explicitly set the pin factory to pigpio
        factory = PiGPIOFactory()
        
        self.servo = Servo(
            SERVO_PIN,
            initial_value=0,
            # These are common pulse widths for standard servos.
            # You might need to fine-tune these for your specific model.
            min_pulse_width=0.0005,  # 0.5ms
            max_pulse_width=0.0025,  # 2.5ms
            pin_factory=factory
        )
        self.last_position = None
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")
        self.servo.value = 0

    def listener_callback(self, msg):
        position = max(-1.0, min(1.0, msg.data))
        # Only update if position changes significantly to reduce calls
        if self.last_position is None or abs(position - self.last_position) > 0.01:
            self.servo.value = position
            self.last_position = position
            self.get_logger().info(f"Set servo to position: {position}")

    def destroy_node(self):
        self.servo.close()
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