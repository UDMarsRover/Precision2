import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

SERVO_PIN = 12
SERVO_FREQ = 50  # Standard servo frequency in Hz

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        self.pwm.start(0)  # Start with a duty cycle of 0

        self.last_position = None
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info("Servo node started with RPi.GPIO. Listening on 'servo_position' topic.")
        self.set_servo_dutycycle(7.5) # Center position

    def set_servo_dutycycle(self, duty_cycle):
        self.pwm.ChangeDutyCycle(duty_cycle)

    def listener_callback(self, msg):
        # Map the ROS position (-1.0 to 1.0) to a duty cycle (2.5 to 12.5)
        # 2.5 is approx. 0 degrees, 12.5 is approx. 180 degrees
        duty_cycle = ((msg.data + 1) / 2) * 10 + 2.5
        
        if self.last_position is None or abs(duty_cycle - self.last_position) > 0.1:
            self.set_servo_dutycycle(duty_cycle)
            self.last_position = duty_cycle
            self.get_logger().info(f"Set servo to duty cycle: {duty_cycle}%")

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
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