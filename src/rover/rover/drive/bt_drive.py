from rover.drive.pro_controller import NintendoProController
from rover.drive.UDMRTMotorSerial import UDMRTMotorSerial
from serial.serialutil import SerialException
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
import time

class BTDrive(Node):
    def __init__(self, serial_port='/dev/serial/by-id/usb-Adafruit_Feather_M4_CAN_CC17951D534837434E202020FF0F291F-if00'):
        super().__init__('bt_drive_node')
        self.get_logger().info("BTDrive node initialized")

        # ROS 2 subscription
        self.controller_sub = self.create_subscription(
            Float32MultiArray,
            'drive_velocities',
            self.control_callback,
            10
        )
        
        # Serial connection
        self.serial_conn = UDMRTMotorSerial(port=serial_port, baudrate=115200)
        if not self.serial_conn.connect():
            raise SerialException("Could not connect to motor controller")
        
        # Nintendo Pro Controller setup
        controller_alive = False
        while not controller_alive:
            try:
                self.controller = NintendoProController()
                controller_alive = True
                self.get_logger().info("Nintendo Pro Controller initialized")
            except Exception as e:
                self.get_logger().info(f"Failed to initialize controller: {e}")
                self.controller = None
                time.sleep(3)
        self.controller.add_analog_callback("LS_x", self.lsx_callback)
        self.controller.add_analog_callback("LS_y", self.lsy_callback)
        
        self.right_velocity = 0.0
        self.left_velocity = 0.0
        self.max_velocity = 300
        self.ls_received = False
        self.lrc_active = False

        self.get_logger().info("Controller setup complete.")

    def control_callback(self, msg):
        self.get_logger().info("LRC active, shutting down bluetooth controller")
        self.lrc_active = True
        # NOTE: The controller will be killed in the main loop's shutdown logic.
        # This callback sets a flag to indicate the main loop should stop using the controller.

    def lsy_callback(self, value):
        if self.ls_received:
            x = getattr(self, 'lsx_value', 0.0)
            y = value
            left_velocity, right_velocity = self.calculate_velocities(x, y)
            self.left_velocity = left_velocity
            self.right_velocity = right_velocity
            velocities = [self.right_velocity] * 3 + [self.left_velocity] * 3
            self.serial_conn.send_velocity_set(velocities)
            self.ls_received = False

    def lsx_callback(self, value):
        self.ls_received = True
        self.lsx_value = value

    def calculate_velocities(self, x, y):
        left_velocity = ((-y) + 0.5 * x) * self.max_velocity
        right_velocity = ((-y) - 0.5 * x) * self.max_velocity
        left_velocity = max(min(left_velocity, self.max_velocity), -self.max_velocity)
        right_velocity = max(min(right_velocity, self.max_velocity), -self.max_velocity)
        self.get_logger().info(f"Calculated velocities: Left: {left_velocity}, Right: {right_velocity}")
        return left_velocity, right_velocity

# Main function to run the node
def main(args=None):
    rclpy.init(args=args)
    bt_drive = BTDrive()
    
    executor = SingleThreadedExecutor()
    executor.add_node(bt_drive)

    try:
        bt_drive.get_logger().info("Starting combined event loop...")
        while rclpy.ok():
            # Process ROS 2 events
            executor.spin_once(timeout_sec=0)
            
            # Check for controller events and process them
            if not bt_drive.lrc_active:
                # You'll need a non-blocking method from your controller library.
                # Assuming `controller.spin_once()` or similar exists.
                # If not, you might need to find an equivalent to process events.
                bt_drive.controller.spin_once()
            else:
                # If LRC is active, we can break out of the controller processing.
                break

            time.sleep(0.01) # Small sleep to prevent busy-waiting
            
    except KeyboardInterrupt:
        bt_drive.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        bt_drive.get_logger().info("Shutting down...")
        if not bt_drive.lrc_active:
            bt_drive.controller.kill()
        bt_drive.serial_conn.disconnect()
        bt_drive.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()