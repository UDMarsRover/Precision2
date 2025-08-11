import pygame
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"

class NintendoProController:
    def __init__(self):
        self.name = "Nintendo Pro Controller"
        self.buttons = {
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "minus": 4,
            "home": 5,
            "plus": 6,
            "LS": 7,
            "RS": 8,
            "L": 9,
            "R": 10,
            "D_up": 11,
            "D_down": 12,
            "D_left": 13,
            "D_right": 14,
            "circle": 15
        }
        self.analogs = {
            "LS_x": 0,
            "LS_y": 1,
            "RS_x": 2,
            "RS_y": 3,
            "ZL": 4,
            "ZR": 5
        }
        self.button_callbacks = {}
        self.analog_callbacks = {}
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected. Please connect your Nintendo Pro Controller via Bluetooth.")
            raise AssertionError("No joystick connected")
        else:
            print(f"Detected {pygame.joystick.get_count()} joystick(s).")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def kill(self):
        pygame.quit()
        raise SystemExit

    def add_button_callback(self, button_name, callback):
        if button_name in self.buttons:
            self.button_callbacks[self.buttons[button_name]] = callback
        else:
            raise ValueError(f"Button {button_name} not found in controller.")
        
    def add_analog_callback(self, stick_name, callback):
        if stick_name in self.analogs:
            self.analog_callbacks[self.analogs[stick_name]] = callback
        else:
            raise ValueError(f"Analog stick {stick_name} not found in controller.")
        
    def run_callbacks(self):
        pygame.event.pump()
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]

        for button_index, value in enumerate(buttons):
            if button_index in self.button_callbacks:
                self.button_callbacks[button_index](value)

        for stick_index, value in enumerate(axes):
            if stick_index in self.analog_callbacks:
                self.analog_callbacks[stick_index](value)

    def spin_once(self):
        self.run_callbacks()

    def run(self):
        try:
            while True:
                self.run_callbacks()
                pygame.time.wait(50)
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            pygame.quit()

    def is_pressed(self, value):
        return value > 0.5


if __name__ == "__main__":
    controller = NintendoProController()
    
    def on_a_pressed(value):
        if controller.is_pressed(value):
            print("A button pressed!")

    def zl_value(value):
        print(f"ZL value: {value}")

    controller.add_button_callback("A", on_a_pressed)
    controller.add_analog_callback("LS_x", zl_value)

    controller.run()
