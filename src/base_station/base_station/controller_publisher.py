import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("ControllerNode initialized and connected to controller.")

        self.drive_pub = self.create_publisher(Float32MultiArray, 'drive_velocities', 10)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected. Please connect your Nintendo Pro Controller via Bluetooth.")
            exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def add_button_callback(self, button_name, callback):
        """
        Add a callback function for a specific button press.
        :param button_name: Name of the button to listen for.
        :param callback: Function to call when the button is pressed.
        """
        if button_name in self.buttons:
            self.callbacks[self.buttons[button_name]] = callback
        else:
            raise ValueError(f"Button {button_name} not found in controller.")
        
    def add_analog_callback(self, stick_name, callback):
        """
        Add a callback function for an analog stick movement.
        :param stick_name: Name of the analog stick to listen for.
        :param callback: Function to call when the stick is moved.
        """
        if stick_name in self.analogs:
            self.callbacks[self.analogs[stick_name]] = callback
        else:
            raise ValueError(f"Analog stick {stick_name} not found in controller.")
        
    def run_callbacks(self):
        """
        Poll the joystick and run callbacks for pressed buttons.
        """
        pygame.event.pump()
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        for button_index in range(len(buttons)):
            if buttons[button_index] and button_index in self.callbacks:
                self.callbacks[button_index](buttons[button_index])

        for stick_index in range(len(axes)):
            if stick_index in self.callbacks:
                self.callbacks[stick_index](axes[stick_index])

    def run(self):
        """
        Main loop to keep the controller running and checking for inputs.
        """
        try:
            while True:
                self.run_callbacks()
                pygame.time.wait(50)  # Poll every 50ms
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            pygame.quit()

    def is_pressed(self, value):
        return value > 0.5

if __name__ == "__main__":
    controller = NintendoProController()
    
    # Example callback functions
    def on_a_pressed(value):
        if controller.is_pressed(value):
            print("A button pressed!")

    def zl_value(value):
        print(f"ZL value: {value}")

    # Register callbacks
    # controller.add_button_callback("A", on_a_pressed)
    controller.add_analog_callback("LS_x", zl_value)
    # controller.add_analog_callback("LS_x", on_ls_moved)

    # Run the controller loop
    controller.run()

class BTDrive:
    def __init__(self, serial_port='/dev/serial/by-id/usb-Adafruit_Feather_M4_CAN_CC17951D534837434E202020FF0F291F-if00'):
        self.serial_conn = UDMRTMotorSerial(port=serial_port, baudrate=115200)
        if not self.serial_conn.connect():
            raise SerialException("Could not connect to motor controller")
        
        self.controller = NintendoProController()
        self.controller.add_analog_callback("LS_x", self.lsx_callback)
        self.controller.add_analog_callback("LS_y", self.lsy_callback)
        
        self.right_velocity = 0.0
        self.left_velocity = 0.0
        self.max_velocity = 300
        self.ls_received = False

        self.controller.run()

    def lsy_callback(self, value):
        # Only calculate velocities if lsc_callback has been called with a new value
        if self.ls_received:
            x = getattr(self, 'lsx_value', 0.0)
            y = value
            # print(f"LS_y value: {y}, LS_x value: {x}")
            left_velocity, right_velocity = self.calculate_velocities(x, y)
            self.left_velocity = left_velocity
            self.right_velocity = right_velocity
            velocities = [self.right_velocity] * 3 + [self.left_velocity] * 3
            # parsed_data = self.serial_conn.spin_once()
            # print(f"Parsed data: {parsed_data}")
            # self.serial_conn.send_velocity_set([0.0, 0.0, 100.0, 100.0, 100.0, 100.0])
            self.serial_conn.send_velocity_set(velocities)
            print(f"Setting velocities: Left: {self.left_velocity}, Right: {self.right_velocity}")
            self.ls_received = False  # Reset flag after processing

    def lsx_callback(self, value):
        # Store the latest x value for use in lsy_callback
        self.ls_received = True
        self.lsx_value = value
        
    def a_callback(self, value):
        parsed_data = self.serial_conn.spin_once()

    def calculate_velocities(self, x, y):
        left_velocity = ((-y) + 0.5 * x) * self.max_velocity
        right_velocity = ((-y) - 0.5 * x) * self.max_velocity
        if left_velocity > self.max_velocity:
            left_velocity = self.max_velocity
        if left_velocity < -self.max_velocity:
            left_velocity = -self.max_velocity
        if right_velocity > self.max_velocity:
            right_velocity = self.max_velocity
        if right_velocity < -self.max_velocity:
            right_velocity = -self.max_velocity
        return left_velocity, right_velocity
        

if __name__ == "__main__":
    try:
        bt_drive = BTDrive()
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"An error occurred: {e}")
