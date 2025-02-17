import serial
from .serial_protocol import construct_message, read_message, CommandType

class MotorController:
    def __init__(self, port, baud_rate=115200, timeout=1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_connection = None

    def connect(self):
        self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
        if self.serial_connection.is_open:
            print(f"Connected to Arduino on port {self.port}")
        else:
            print(f"Failed to connect to Arduino on port {self.port}")

    def disconnect(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print(f"Disconnected from Arduino on port {self.port}")

    def send_command(self, command: CommandType, data=None):
        if self.serial_connection and self.serial_connection.is_open:
            message = construct_message(command.value, data)
            self.serial_connection.write(message)

    def toggle_led(self):
        self.send_command(CommandType.LED, data='T')

    def set_led(self, state):
        self.send_command(CommandType.LED, data='1' if state else '0')

    def get_led_status(self):
        self.send_command(CommandType.STATUS, data='L')
        return self.read_response() == '1'

    def read_response(self):
        if self.serial_connection and self.serial_connection.is_open:
            response = self.serial_connection.readline().decode().strip()
            return response
        return None

    def __del__(self):
        self.disconnect()