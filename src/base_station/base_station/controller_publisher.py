import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import threading

DRIVE_MODE = 0
ARM_MODE = 1

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("ControllerNode initialized and connected to controller.")

        self.drive_pub = self.create_publisher(Float32MultiArray, 'drive_velocities', 10)

        self.servo_pub = self.create_publisher(Float32, 'servo_position', 10)

        self.camera_yaw_pub = self.create_publisher(Float32, 'camera_yaw', 10)

        self.buttons = {
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "LB": 4,
            "RB": 5,
            "BACK": 6,
            "START": 7,
        }

        self.axes = {
            "LSX": 0,
            "LSY": 1,
            "RSX": 3,
            "RSY": 4,
            "LT": 2,
            "RT": 5,
        }

        self.hats = {
            "X": 0,
            "Y": 1,
        }

        self.button_callbacks = {}
        self.axis_callbacks = {}
        self.hat_callbacks = {}

        self.right_velocity = 0.0
        self.left_velocity = 0.0
        self.max_velocity = 250
        self.ls_received = False

        # debounce mechanism
        self.last_start_condition = 0

        self.mode = DRIVE_MODE  # Default mode is DRIVE_MODE

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected. Please connect your Nintendo Pro Controller via Bluetooth.")
            exit(1)

        self.add_axis_callback("LSX", self.lsx_callback)
        self.add_axis_callback("LSY", self.lsy_callback)
        self.add_axis_callback("RSX", self.rsx_callback)
        self.add_axis_callback("RSY", self.rsy_callback)
        self.add_button_callback("START", self.start_callback)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def add_button_callback(self, button_name, callback):
        """
        Add a callback function for a specific button press.
        :param button_name: Name of the button to listen for.
        :param callback: Function to call when the button is pressed.
        """
        if button_name in self.buttons:
            self.button_callbacks[self.buttons[button_name]] = callback
        else:
            raise ValueError(f"Button {button_name} not found in controller.")

    def add_axis_callback(self, axis_name, callback):
        """
        Add a callback function for an axis movement.
        :param axis_name: Name of the axis to listen for.
        :param callback: Function to call when the axis is moved.
        """
        if axis_name in self.axes:
            self.axis_callbacks[self.axes[axis_name]] = callback
        else:
            raise ValueError(f"Axis {axis_name} not found in controller.")
        
    def add_hat_callback(self, hat_name, callback):
        """
        Add a callback function for a hat movement.
        :param hat_name: Name of the hat to listen for.
        :param callback: Function to call when the hat is moved.
        """
        if hat_name in self.hats:
            self.hat_callbacks[self.hats[hat_name]] = callback
        else:
            raise ValueError(f"Hat {hat_name} not found in controller.")
        
    def run_callbacks(self):
        """
        Poll the joystick and run callbacks for pressed buttons.
        """
        pygame.event.pump()
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
        for button_index in range(len(buttons)):
            if button_index in self.button_callbacks:
                self.button_callbacks[button_index](buttons[button_index])

        for axis_callback in range(len(axes)):
            if axis_callback in self.axis_callbacks:
                self.axis_callbacks[axis_callback](axes[axis_callback])

        for hat_index in range(len(hats)):
            if hat_index in self.hat_callbacks:
                self.hat_callbacks[hat_index](hats[hat_index])

    def lsy_callback(self, value):
        # Only calculate velocities if lsc_callback has been called with a new value
        if self.mode == DRIVE_MODE and self.ls_received:
            x = getattr(self, 'lsx_value', 0.0)
            y = value
            # print(f"LS_y value: {y}, LS_x value: {x}")
            left_velocity, right_velocity = self.calculate_velocities(x, y)
            self.left_velocity = left_velocity
            self.right_velocity = right_velocity
            self.send_drive_velocities()
            self.ls_received = False  # Reset flag after processing

    def lsx_callback(self, value):
        # Store the latest x value for use in lsy_callback
        if self.mode == DRIVE_MODE:
            self.ls_received = True
            self.lsx_value = value

    def rsy_callback(self, value):
        self.servo_pub.publish(Float32(data=value))

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
        return float(left_velocity), float(right_velocity)

    def send_drive_velocities(self):
        """
        Publishes the current drive velocities to the 'drive_velocities' topic.
        """
        msg = Float32MultiArray()
        msg.data = [self.left_velocity, self.right_velocity]
        self.drive_pub.publish(msg)

    def start_callback(self, value):
        # print(f"Start button pressed with value: {value}")
        if self.last_start_condition < value:
            if self.mode == DRIVE_MODE:
                self.mode = ARM_MODE
                self.get_logger().info("Switched to ARM_MODE.")
            else:
                self.mode = DRIVE_MODE
                self.get_logger().info("Switched to DRIVE_MODE.")
        self.last_start_condition = value

    def rsx_callback(self, value):
        """
        Callback for the right stick x-axis movement.
        Publishes the value to the 'camera_yaw' topic.
        """
        self.camera_yaw_pub.publish(Float32(data=value))
        # print(f"Camera yaw set to: {value}")

    def run(self):
        """
        Main loop to keep the controller running and checking for inputs.
        """
        try:
            while True:
                self.run_callbacks()
                pygame.time.wait(100)  # Poll every 20ms
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            pygame.quit()

    def is_pressed(self, value):
        return value > 0.5

if __name__ == "__main__":
    rclpy.init()
    controller = ControllerNode()
    
    # Example callback functions
    def on_a_pressed(value):
        if controller.is_pressed(value):
            print("A button pressed!")

    def on_trigger_moved(value):
        print(f"Trigger moved: {value}")
    
    # controller.add_analog_callback("LS_x", zl_value)
    # controller.add_analog_callback("LS_x", on_ls_moved)

    # Run the controller loop
    controller.run()
