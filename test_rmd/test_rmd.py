#!/bin/python3

import myactuator_rmd_py as rmd
from time import sleep

# Initialize the CAN driver
try:
    driver = rmd.CanDriver("can0")
except Exception as e:
    print(f"Failed to initialize CAN driver: {e}")
    exit(1)

# Create motor instances
motors = [
    rmd.ActuatorInterface(driver, 1),  # Motor 1
    rmd.ActuatorInterface(driver, 2),  # Motor 2
    rmd.ActuatorInterface(driver, 3),  # Motor 3
    rmd.ActuatorInterface(driver, 4),  # Motor 4
    rmd.ActuatorInterface(driver, 5),  # Motor 5
    rmd.ActuatorInterface(driver, 6),  # Motor 6
    # rmd.ActuatorInterface(driver, 7)  # Motor 7
]

# Print initial positions
print("Initial Multi-Turn Angles:")
for motor in motors:
    try:
        print(f"Motor {motor.getCanId()}: {motor.getMultiTurnAngle()}")
        sleep(0.02)
    except rmd.ProtocolException as e:
        print(f"Error reading angle for Motor {motor.getCanId()}: {e}")

# # Define home positions
# home_positions = [341, 150, 53, 344, 285, 356]

# def home_motors(motors, home_positions):
#     for motor, home_position in zip(motors, home_positions):
#         retries = 0
#         while retries < 5:  # Limit retries to avoid infinite loop
#             try:
#                 current_pos = motor.getMultiTurnAngle()
#                 print(f"Motor {motor.getCanId()} Current Position: {current_pos}")

#                 if abs(current_pos - home_position) <= 0.1:
#                     print(f"Motor {motor.getCanId()} is at home position.")
#                     break

#                 # Send position command
#                 motor.sendPositionAbsoluteSetpoint(home_position, 50)
#                 sleep(1)
#             except rmd.ProtocolException as e:
#                 print(f"Protocol Exception for Motor {motor.getCanId()}: {e}")
#                 retries += 1
#                 sleep(0.5)
#             except Exception as e:
#                 print(f"Unexpected error with Motor {motor.getCanId()}: {e}")
#                 retries += 1

#         if retries >= 5:
#             print(f"Motor {motor.getCanId()} failed to reach home position after retries.")

#     print("All motors homed and zeroed.")

# # Home the motors
# home_motors(motors, home_positions)
