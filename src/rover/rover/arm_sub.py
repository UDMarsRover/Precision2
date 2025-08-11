import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import serial

class ArmSerialWriter(Node):
    def __init__(self):
        super().__init__('arm_serial_writer')
        self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
        self.subscription = self.create_subscription(
            Int8,
            'arm_codes',
            self.callback,
            10
        )
        self.get_logger().info("arm_serial_writer node started, listening to 'arm_codes' topic.")

    def callback(self, msg):
        data = bytes([msg.data])
        self.ser.write(data)
        self.get_logger().info(f"Sent {msg.data} to serial port.")

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
        pass
    finally:
        node.get_logger().info("Destroying node and closing serial port.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()