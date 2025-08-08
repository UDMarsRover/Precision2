import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import gpiod
import threading
import time
from collections import deque

SERVO_PIN = 12
MIN_PULSE = 500   # microseconds
MAX_PULSE = 2500  # microseconds
MIN_POSITION = -0.7  # New minimum position, as a percentage of the total range (-1.0 to 1.0)
MAX_POSITION = 0.5   # New maximum position, as a percentage of the total range (-1.0 to 1.0)

# The period for a 50Hz PWM signal is 20,000 microseconds
PWM_PERIOD_US = 20000

# Moving average filter settings
SMOOTHING_WINDOW_SIZE = 5

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # Initialize gpiod
        try:
            self.chip = gpiod.Chip('gpiochip0')
            self.line = self.chip.get_line(SERVO_PIN)
            self.line.request(consumer='servo-control', type=gpiod.LINE_REQ_DIR_OUT)
            self.get_logger().info(f"Successfully configured GPIO pin {SERVO_PIN}.")
        except Exception as e:
            self.get_logger().error(f"Error configuring gpiod: {e}")
            raise Exception("gpiod configuration failed")

        self.target_pulse_width = 1500
        self.last_pulse_width = self.target_pulse_width
        self.thread_running = True

        # Initialize the moving average filter
        self.position_history = deque(maxlen=SMOOTHING_WINDOW_SIZE)

        self.subscription = self.create_subscription(
            Float32,
            'servo_position',
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Servo node started. Listening on 'servo_position' topic.")

        # Start a dedicated thread for software PWM
        self.pwm_thread = threading.Thread(target=self.pwm_loop)
        self.pwm_thread.start()

    def pwm_loop(self):
        while self.thread_running:
            # Get the current pulse width
            pulse_width = self.target_pulse_width

            # High time is the pulse width
            high_time_us = pulse_width
            # Low time is the period minus the high time
            low_time_us = PWM_PERIOD_US - high_time_us

            # Perform the software PWM pulse
            self.line.set_value(1)
            time.sleep(high_time_us / 1000000.0)
            self.line.set_value(0)
            time.sleep(low_time_us / 1000000.0)

    def listener_callback(self, msg):
        # Add the new data to the smoothing window
        self.position_history.append(msg.data)

        # Calculate the average of the values in the window
        smoothed_data = sum(self.position_history) / len(self.position_history)

        # Enforce the new minimum position
        smoothed_data = max(MIN_POSITION, smoothed_data)
        smoothed_data = min(MAX_POSITION, smoothed_data)

        # Convert the smoothed data to a pulse width
        pulse_width = int(((smoothed_data + 1) / 2) * (MAX_PULSE - MIN_PULSE) + MIN_PULSE)
        pulse_width = max(MIN_PULSE, min(MAX_PULSE, pulse_width))

        # Update the target pulse width only if there is a significant change
        if abs(pulse_width - self.last_pulse_width) > 10:
            self.target_pulse_width = pulse_width
            self.last_pulse_width = pulse_width
            self.get_logger().info(f"Smoothed position: {smoothed_data:.2f}, Set servo to pulse width: {pulse_width}us")

    def destroy_node(self):
        # Stop the PWM thread
        self.thread_running = False
        self.pwm_thread.join()

        # Clean up gpiod
        self.line.set_value(0)
        self.line.release()
        self.chip.close()
        self.get_logger().info("GPIO resources released.")
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