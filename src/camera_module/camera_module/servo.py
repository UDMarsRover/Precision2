import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Servo

SERVO_PIN = 12
SMOOTH_STEP = 0.02  # Step size for smoothing
SMOOTH_INTERVAL = 0.02  # Seconds between steps

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.servo = Servo(SERVO_PIN)
        self.target_position = 0.0
        self.current_position = 0.0
        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.timer = self.create_timer(SMOOTH_INTERVAL, self.smooth_move)
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")

    def listener_callback(self, msg):
        # Clamp input to [-1, 1]
        self.target_position = max(-1.0, min(1.0, msg.data))
        self.get_logger().info(f"Received target position: {self.target_position}")

    def smooth_move(self):
        if abs(self.current_position - self.target_position) < SMOOTH_STEP:
            self.current_position = self.target_position
        else:
            direction = 1 if self.target_position > self.current_position else -1
            self.current_position += direction * SMOOTH_STEP
        self.servo.value = self.current_position

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
