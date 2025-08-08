import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from UDMRTMotorSerial import UDMRTMotorSerial
from serial.serialutil import SerialException
class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.serial_port = '/dev/serial/by-id/usb-Adafruit_Feather_M4_CAN_CC17951D534837434E202020FF0F291F-if00'
        self.serial_conn = UDMRTMotorSerial(port=self.serial_port, baudrate=115200)
        if not self.serial_conn.connect():
            raise SerialException("Could not connect to motor controller")
        
        self.get_logger().info("DriveNode initialized and connected to motor controller.")

        self.control_sub = self.create_subscription(
            Float32MultiArray,
            'drive_velocities',
            self.control_callback,
            10
        )

    def control_callback(self, msg):
        # Handle incoming control messages
        if self.serial_conn:
            velocities = [msg.data[1]] * 3 + [msg.data[0]] * 3
            self.serial_conn.send_velocity_set(velocities)
            self.get_logger().info(f"Set velocities: Left: {msg.data[0]}, Right: {msg.data[1]}")
        
def main(args=None):
    rclpy.init(args=args)
    drive_node = DriveNode()
    rclpy.spin(drive_node)
    drive_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
