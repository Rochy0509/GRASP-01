import myactuator_rmd_py as rmd
import time

# Initialize CAN driver and motor interfaces
driver = rmd.CanDriver("can0")
motors = [
    rmd.ActuatorInterface(driver, 1),  # Motor 1
    rmd.ActuatorInterface(driver, 2),  # Motor 2
    rmd.ActuatorInterface(driver, 3),  # Motor 3
    rmd.ActuatorInterface(driver, 4),  # Motor 4
    rmd.ActuatorInterface(driver, 5),  # Motor 5
    rmd.ActuatorInterface(driver, 6),  # Motor 5
    # rmd.ActuatorInterface(driver, 7),  # Motor 5

]

# Homing function that takes a list of home positions for each motor
def home_motors(motors, home_positions):
    
    for motor, home_position in zip(motors, home_positions):
        while True:
            current_pos = motor.getMultiTurnAngle()
            print(f"Current Position {motor.getCanId()}: {current_pos}")

            if abs(current_pos - home_position) <= 0.1:
                print(f"Motor {motor.getCanId()} is at home position.")
                break

            motor.sendPositionAbsoluteSetpoint(home_position, 10)
            time.sleep(0.1)

        # motor.setEncoderZero(1)
    print("All motors homed and zeroed")

# Example home positions for each motor
home_positions = [340, 210, 308, 445, 290, 352]  # Adjust these as needed for each motor's unique home position

# Home all motors
home_motors(motors, home_positions)

# Simple user input to test motors
try:
    while True:
        # Ask user for motor ID and target angle
        motor_id = int(input("Enter motor ID (1, 2, or 3) to control: "))
        angle = float(input("Enter target angle (0-90): "))

        # Constrain motor ID to range of available motors
        if motor_id < 1 or motor_id > len(motors):
            print("Invalid motor ID. Please enter 1, 2, or 3.")
            continue

        # Send position command to selected motor
        motors[motor_id - 1].sendPositionAbsoluteSetpoint(angle, 10)
        print(f"Motor {motor_id} moved to {angle}°")

        # Option to exit loop
        cont = input("Continue? (y/n): ")
        if cont.lower() != 'y':
            break

except KeyboardInterrupt:
    print("Exiting program.")
