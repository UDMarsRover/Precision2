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

        # Serial connection setup
        self.serial_conn = UDMRTMotorSerial(port=serial_port, baudrate=115200)
        if not self.serial_conn.connect():
            raise SerialException("Could not connect to motor controller")

        # Attempt to set up the controller by waiting for it to be ready.
        # This will block until the controller is found.
        self.controller = self.setup_controller()

        # The rest of __init__ is unchanged as it only runs if the controller is ready.
        self.controller.add_analog_callback("LS_x", self.lsx_callback)
        self.controller.add_analog_callback("LS_y", self.lsy_callback)
        self.controller.add_button_callback("D_up", self.increment_mode)
        self.controller.add_button_callback("D_down", self.decrement_mode)
        self.get_logger().info("Controller setup complete.")

        # ROS 2 subscription
        self.controller_sub = self.create_subscription(
            Float32MultiArray,
            'drive_velocities',
            self.control_callback,
            10
        )
        
        self.right_velocity = 0.0
        self.left_velocity = 0.0
        self.max_velocity = 300
        self.ls_received = False
        self.lrc_active = False

        self.last_a = 0
        self.last_b = 0

        self.mode = 0
        self.max_vels = [200, 400, 600, 800, 1000]

    def setup_controller(self):
        """
        Attempts to initialize the Nintendo Pro Controller in a blocking, indefinite loop.
        It will only return once a controller is successfully initialized.
        """
        while True:
            try:
                controller = NintendoProController()
                self.get_logger().info("Nintendo Pro Controller initialized")
                return controller
            except Exception as e:
                self.get_logger().info(f"Failed to initialize controller: {e}. Retrying in 3 seconds...")
                time.sleep(3)
    def increment_mode(self, value):
        """
        Increment the mode and update the maximum velocities accordingly.
        """
        if value > 0 and self.last_a != value:
            self.get_logger().info("Incrementing mode")
            self.mode = (self.mode + 1) % len(self.max_vels)
            self.max_velocity = self.max_vels[self.mode]
            self.get_logger().info(f"Mode changed to {self.mode}, max velocity set to {self.max_velocity}")
            self.last_a = value 

    def decrement_mode(self, value):
        """
        Decrement the mode and update the maximum velocities accordingly.
        """
        if value > 0 and self.last_b != value:
            self.mode = (self.mode - 1) % len(self.max_vels)
            self.max_velocity = self.max_vels[self.mode]
            self.get_logger().info(f"Mode changed to {self.mode}, max velocity set to {self.max_velocity}")
            self.last_b = value

    def control_callback(self, msg):
        self.get_logger().info("LRC active, shutting down bluetooth controller")
        self.lrc_active = True

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

def main(args=None):
    rclpy.init(args=args)
    bt_drive = None
    try:
        bt_drive = BTDrive()
        executor = SingleThreadedExecutor()
        executor.add_node(bt_drive)
        bt_drive.get_logger().info("Starting combined event loop...")

        while rclpy.ok():
            executor.spin_once(timeout_sec=0)
            
            if not bt_drive.lrc_active and bt_drive.controller is not None:
                try:
                    bt_drive.controller.spin_once()
                except Exception as e:
                    bt_drive.get_logger().error(f"Controller runtime error: {e}")
                    bt_drive.lrc_active = True
            elif bt_drive.lrc_active:
                break

            time.sleep(0.01)
            
    except KeyboardInterrupt:
        if bt_drive:
            bt_drive.get_logger().info("Keyboard interrupt received, shutting down.")
    except (RuntimeError, SerialException) as e:
        if bt_drive:
            bt_drive.get_logger().fatal(f"Initialization failed: {e}")
        else:
            print(f"Initialization failed: {e}")
    finally:
        if bt_drive:
            bt_drive.get_logger().info("Shutting down...")
            if bt_drive.controller is not None:
                bt_drive.controller.kill()
            if bt_drive.serial_conn:
                bt_drive.serial_conn.disconnect()
            bt_drive.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()