import pygame
import sys

def main():
    pygame.init()
    pygame.joystick.init()



    if pygame.joystick.get_count() == 0:
        print("No game controller detected over USB.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected controller: {joystick.get_name()}")

    print("Press Ctrl+C to exit.")
    try:
        while True:
            pygame.event.pump()
            axes = [round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())]
            buttons = [round(joystick.get_button(i), 2) for i in range(joystick.get_numbuttons())]
            hats = [tuple(round(h, 2) for h in joystick.get_hat(i)) for i in range(joystick.get_numhats())]

            print(f"Axes: {axes} | Buttons: {buttons} | Hats: {hats}", end='\r')
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()

"""
mode off:
axes
lsx 0
lsy 1
rsx 3
rsy 4

lt 2
rt 5

buttons
a 0
b 1
x 2
y 3

lb 4
rb 5

back 6
start 7

hats
x 00
y 01

mode on:
lllo


"""