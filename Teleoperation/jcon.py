import pygame
import time

# --- DESCRIPTION ---
# This script uses the pygame library to detect and read input from a connected
# USB game controller. It initializes pygame, finds the first available joystick,
# and enters a loop to continuously print out events for button presses/releases,
# analog stick movements (axes), and directional pad (hat) changes.

# --- SCRIPT ---

# Initialize pygame
pygame.init()

# Check if any joysticks are connected
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected. Please connect a controller.")
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Initialized Joystick: {joystick.get_name()}")
print(f"Number of Axes: {joystick.get_numaxes()}")
print(f"Number of Buttons: {joystick.get_numbuttons()}")
print(f"Number of Hats: {joystick.get_numhats()}")

# Main loop to process events
try:
    while True:
        # Pump events to get the latest state
        pygame.event.pump()

        # --- Read Button Presses ---
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                print(f"Button {i} pressed")

        # --- Read Analog Sticks (Axes) ---
        # Axes are reported as values between -1.0 and 1.0.
        # A small "deadzone" is used to prevent printing minimal drift.
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            if abs(axis_value) > 0.1: # Deadzone of 0.1
                print(f"Axis {i} value: {axis_value:.3f}")

        # --- Read D-Pad (Hat) ---
        # Hats are reported as a tuple (x, y), e.g., (1, 0) for right.
        for i in range(joystick.get_numhats()):
            hat_value = joystick.get_hat(i)
            if hat_value != (0, 0):
                print(f"Hat {i} value: {hat_value}")
        
        # Add a small delay to prevent spamming the console
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting program.")
    pygame.quit()