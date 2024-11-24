import myactuator_rmd_py as rmd
import time
import threading
import matplotlib.pyplot as plt

# Initialize CAN driver and motor interfaces
driver = rmd.CanDriver("can0")
motors = [
    rmd.ActuatorInterface(driver, 1),  # Motor 1
    rmd.ActuatorInterface(driver, 2),  # Motor 2
    rmd.ActuatorInterface(driver, 3),  # Motor 3
    rmd.ActuatorInterface(driver, 4),  # Motor 4
]

# Assign explicit IDs to motors (1, 2, 3, 4)
motor_ids = [1, 2, 3, 4]  # IDs corresponding to motor order in the array

# Shared data for plotting
time_data = []
position_data = {motor_id: [] for motor_id in motor_ids}  # Initialize position data for each motor
start_time = time.time()
data_lock = threading.Lock()

# Background data collection function
def collect_motor_data(motors, motor_ids):
    global time_data, position_data
    while True:
        current_time = time.time() - start_time
        with data_lock:
            # Add time data
            time_data.append(current_time)

            # Add position data for each motor
            for motor, motor_id in zip(motors, motor_ids):
                try:
                    current_position = motor.getMultiTurnAngle()
                    position_data[motor_id].append(current_position)
                except Exception as e:
                    print(f"Error reading Motor {motor_id}: {e}")
                    position_data[motor_id].append(0)  # Add default value if error

            # Trim data to maintain consistency
            if len(time_data) > 100:
                time_data = time_data[-100:]
                for motor_id in position_data.keys():
                    position_data[motor_id] = position_data[motor_id][-100:]
        time.sleep(0.1)

# Start data collection thread
data_thread = threading.Thread(target=collect_motor_data, args=(motors, motor_ids), daemon=True)
data_thread.start()

# Plotting function in the main thread
def plot_motor_positions():
    plt.ion()
    fig, ax = plt.subplots()
    while True:
        with data_lock:
            ax.clear()
            for motor_id, positions in position_data.items():
                # Ensure positions length matches time_data length
                if len(positions) == len(time_data):
                    ax.plot(time_data, positions, label=f"Motor {motor_id}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position")
            ax.legend()
        plt.pause(0.1)

# Homing function that takes a list of home positions for each motor
def home_motors(motors, home_positions, motor_ids):
    for motor, home_position, motor_id in zip(motors, home_positions, motor_ids):
        while True:
            current_pos = motor.getMultiTurnAngle()
            print(f"Current Position Motor {motor_id}: {current_pos}")

            if abs(current_pos - home_position) <= 0.1:
                print(f"Motor {motor_id} is at home position.")
                break

            motor.sendPositionAbsoluteSetpoint(home_position, 10)
            time.sleep(0.1)
    print("All motors homed and zeroed")

# Example home positions for each motor
home_positions = [341, 150, 339, 240]  # Adjust these as needed for each motor's unique home position

# Start the plotting in the main thread
plot_thread = threading.Thread(target=plot_motor_positions, daemon=True)
plot_thread.start()

# Home all motors
home_motors(motors, home_positions, motor_ids)

# Simple user input to test motors
try:
    while True:
        # Ask user for motor ID and target angle
        motor_id = int(input("Enter motor ID (1, 2, 3, or 4) to control: "))
        angle = float(input("Enter target angle (0-90): "))

        # Constrain motor ID to range of available motors
        if motor_id < 1 or motor_id > len(motors):
            print("Invalid motor ID. Please enter 1, 2, 3, or 4.")
            continue

        # Send position command to selected motor
        motors[motor_id - 1].sendPositionAbsoluteSetpoint(angle, 30)
        print(f"Motor {motor_id} moved to {angle}°")

        # Option to exit loop
        cont = input("Continue? (y/n): ")
        if cont.lower() != 'y':
            break

except KeyboardInterrupt:
    print("Exiting program.")
