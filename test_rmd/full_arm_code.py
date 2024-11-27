import myactuator_rmd_py as rmd
import time
import threading
import matplotlib.pyplot as plt
import mediapipe_test as mp
import cv2
import numpy as np
import signal
import sys

# Initialize MediaPipe and RealSense
pipeline = mp.initialize_pipeline()
mp_holistic = mp.mp.solutions.holistic
holistic = mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Initialize Motors
driver = rmd.CanDriver("can0")
motors = [
    rmd.ActuatorInterface(driver, 1),  # Motor 1
    rmd.ActuatorInterface(driver, 2),  # Motor 2
    rmd.ActuatorInterface(driver, 3),  # Motor 3
    rmd.ActuatorInterface(driver, 4),  # Motor 4
    rmd.ActuatorInterface(driver, 5),  # Motor 5
    rmd.ActuatorInterface(driver, 6),  # Motor 6
]

motor_ids = [1, 2, 3, 4, 5, 6]

# Define motor limits (min, max) for safety checks
limits = {
    1: (338, 360),  # Motor 1 range
    2: (180, 210),  # Motor 2 range (counterclockwise only)
    3: (213, 377),  # Motor 3 range
    4: (735, 835),  # Motor 4 range
    5: (198, 378),  # Motor 5 range
    6: (300, 410),  # Motor 6 range
}

# Define expected home positions for motors
home_positions = [340, 210, 290, 735, 290, 350]

time_data = []
position_data = {motor_id: [] for motor_id in motor_ids}
start_time = time.time()
data_lock = threading.Lock()

def shutdownMotor():
    """
    Moves motors to home positions sequentially and shuts them down.
    """
    print("\nShutting down motors...")
    for motor, home_position, motor_id in zip(motors, home_positions, motor_ids):
        try:
            print(f"Homing Motor {motor_id} to position: {home_position}")
            motor.sendPositionAbsoluteSetpoint(home_position, 10)
            
            # Wait for the motor to reach its home position
            while True:
                current_position = motor.getMultiTurnAngle()
                if abs(current_position - home_position) <= 0.1:  # Tolerance for homing
                    print(f"Motor {motor_id} reached home position: {home_position}")
                    break
                print(f"Motor {motor_id} current position: {current_position}. Homing in progress...")
                time.sleep(0.1)  # Check position periodically

            print(f"Motor {motor_id} is now at home position. Proceeding to shutdown...")
        except Exception as e:
            print(f"Error during shutdown of Motor {motor_id}: {e}")
    
    # After all motors are homed, shut them down
    for motor, motor_id in zip(motors, motor_ids):
        try:
            motor.shutdownMotor()
            print(f"Motor {motor_id} powered down.")
        except Exception as e:
            print(f"Error powering down Motor {motor_id}: {e}")
    
    print("All motors homed and powered down.")

    

# Graceful exit handler
def signal_handler(sig, frame):
    print("\nCTRL+C detected. Cleaning up and shutting down...")
    shutdownMotor()
    pipeline.stop()
    cv2.destroyAllWindows()
    sys.exit(0)

# Register signal handler for CTRL+C
signal.signal(signal.SIGINT, signal_handler)

def validate_home_positions(motors, home_positions, motor_ids, limits):
    """
    Validates that each motor is within its limits and at its expected home position.
    Prompts the user to confirm recalibration if necessary.
    """
    for motor, home_position, motor_id in zip(motors, home_positions, motor_ids):
        try:
            current_position = motor.getMultiTurnAngle()
            min_limit, max_limit = limits[motor_id]

            print(f"Motor {motor_id} current position: {current_position}")
            
            # Check if within limits
            if not (min_limit <= current_position <= max_limit):
                print(f"Motor {motor_id} out of bounds! Current position: {current_position}")
                user_decision = input(f"Do you want to recalibrate Motor {motor_id}? (y/n): ").strip().lower()
                if user_decision == 'y':
                    recalibrate_motor(motor, motor_id, min_limit, max_limit)
                else:
                    print(f"Skipping recalibration for Motor {motor_id}. Proceeding with caution.")

            # Check if at expected home position
            elif abs(current_position - home_position) > 0.1:
                print(f"Motor {motor_id} is not at the expected home position ({home_position}).")
                print(f"Current position: {current_position}")
                user_decision = input(f"Do you want to recalibrate Motor {motor_id}? (y/n): ").strip().lower()
                if user_decision == 'y':
                    recalibrate_motor(motor, motor_id, min_limit, max_limit)
                else:
                    print(f"Skipping recalibration for Motor {motor_id}. Proceeding with caution.")
            else:
                print(f"Motor {motor_id} is within range and at the correct home position.")
        except Exception as e:
            print(f"Error validating Motor {motor_id}: {e}")

def recalibrate_motor(motor, motor_id, min_limit, max_limit):
    """
    Recalibrates a motor to ensure it is within limits and properly positioned.
    """
    print(f"Recalibrating Motor {motor_id}...")
    while True:
        current_position = motor.getMultiTurnAngle()
        if min_limit <= current_position <= max_limit:
            print(f"Motor {motor_id} recalibrated to position {current_position}.")
            break
        target_position = (min_limit + max_limit) / 2
        print(f"Moving Motor {motor_id} to recalibration position: {target_position}")
        motor.sendPositionAbsoluteSetpoint(target_position, 10)
        time.sleep(0.1)

def collect_motor_data(motors, motor_ids):
    global time_data, position_data
    while True:
        current_time = time.time() - start_time
        with data_lock:
            time_data.append(current_time)

            for motor, motor_id in zip(motors, motor_ids):
                try:
                    current_position = motor.getMultiTurnAngle()
                    position_data[motor_id].append(current_position)
                except Exception as e:
                    print(f"Error reading Motor {motor_id}: {e}")
                    position_data[motor_id].append(0)

            if len(time_data) > 100:
                time_data = time_data[-100:]
                for motor_id in position_data.keys():
                    position_data[motor_id] = position_data[motor_id][-100:]
        time.sleep(0.1)

def plot_motor_positions():
    plt.ion()
    fig, ax = plt.subplots()
    while True:
        with data_lock:
            ax.clear()
            for motor_id, positions in position_data.items():
                if len(positions) == len(time_data):
                    ax.plot(time_data, positions, label=f"Motor {motor_id}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position")
            ax.legend()
        plt.pause(0.1)

def home_motors(motors, home_positions, motor_ids):
    """
    Homes the motors to their expected positions with safety checks.
    """
    for motor, home_position, motor_id in zip(motors, home_positions, motor_ids):
        while True:
            current_pos = motor.getMultiTurnAngle()
            print(f"Current Position Motor {motor_id}: {current_pos}")

            if abs(current_pos - home_position) <= 0.1:
                print(f"Motor {motor_id} is at home position.")
                break

            motor.sendPositionAbsoluteSetpoint(home_position, 10)
            time.sleep(0.1)
    print("All motors homed and zeroed.")

def motion_test(finger_state):
    """
    Perform motor actions based on the finger state.
    :param finger_state: Dictionary indicating which fingers are up.
    """
    # Define poses
    picking_pose = [340, 210, 290, 835, 290, 350]  # Picking pose values
    dropping_pose = [360, 180, 220, 835, 312, 350]  # Dropping pose values

    # Ensure all required keys are present in finger_state with default values
    finger_state = {
        "Thumb": finger_state.get("Thumb", False),
        "Index": finger_state.get("Index", False),
        "Middle": finger_state.get("Middle", False),
        "Ring": finger_state.get("Ring", False),
        "Pinky": finger_state.get("Pinky", False),
        "FullyClosed": finger_state.get("FullyClosed", False)
    }

    # Fully closed hand: Go to home position
    if finger_state["FullyClosed"]:
        print("Hand fully closed. Moving motors to home positions.")
        for motor, home_position, motor_id in zip(motors, home_positions, motor_ids):
            current_position = motor.getMultiTurnAngle()
            if abs(current_position - home_position) > 0.1:  # Move only if not at home
                print(f"Moving Motor {motor_id} to home position: {home_position}")
                motor.sendPositionAbsoluteSetpoint(home_position, 10)
            else:
                print(f"Motor {motor_id} is already at home position.")

    # Thumb + Index + Middle: Move to dropping pose
    elif finger_state["Thumb"] and finger_state["Index"] and finger_state["Middle"]:
        print("Thumb, Index, and Middle fingers are up. Preparing to move to dropping pose.")

        # Verify the arm is at the picking pose
        at_picking_pose = all(
            abs(motor.getMultiTurnAngle() - pick_position) <= 0.1
            for motor, pick_position in zip(motors, picking_pose)
        )
        if not at_picking_pose:
            print("Arm is not in the picking pose. Aborting dropping motion for safety.")
            return

        print("Arm is at picking pose. Proceeding to move to dropping pose.")

        # Move motors to dropping pose in the specified order
        for motor_id in [1, 2, 3, 5]:  # Order: Motor 1, 2, 3, 5
            motor = motors[motor_id - 1]
            drop_position = dropping_pose[motor_id - 1]
            print(f"Moving Motor {motor_id} to dropping position: {drop_position}")
            motor.sendPositionAbsoluteSetpoint(drop_position, 7.5)

    # Index + Middle: Return to picking pose from dropping pose
    elif finger_state["Index"] and finger_state["Middle"] and not finger_state["Thumb"]:
        print("Index and Middle fingers are up. Preparing to return to picking pose.")

        # Verify the arm is at the dropping pose
        at_dropping_pose = all(
            abs(motor.getMultiTurnAngle() - drop_position) <= 0.1
            for motor, drop_position in zip(motors, dropping_pose)
        )
        if not at_dropping_pose:
            print("Arm is not in the dropping pose. Aborting return motion for safety.")
            return

        print("Arm is at dropping pose. Proceeding to return to picking pose.")

        # Move motors to picking pose in the specified reverse order
        for motor_id in [3, 1, 2, 5]:  # Order: Motor 3, 1, 2, 5
            motor = motors[motor_id - 1]
            pick_position = picking_pose[motor_id - 1]
            print(f"Moving Motor {motor_id} back to picking position: {pick_position}")
            motor.sendPositionAbsoluteSetpoint(pick_position, 7.5)

    # Index: Move directly to picking pose
    elif finger_state["Index"]:
        print("Index finger is up. Moving motors to picking pose.")
        for motor, pick_position, motor_id in zip(motors, picking_pose, motor_ids):
            current_position = motor.getMultiTurnAngle()
            if abs(current_position - pick_position) > 0.1:  # Move only if not at picking pose
                print(f"Moving Motor {motor_id} to picking pose: {pick_position}")
                motor.sendPositionAbsoluteSetpoint(pick_position, 10)
            else:
                print(f"Motor {motor_id} is already at picking pose.")


plot_thread = threading.Thread(target=plot_motor_positions, daemon=True)
plot_thread.start()

# Validate and home motors
validate_home_positions(motors, home_positions, motor_ids, limits)
home_motors(motors, home_positions, motor_ids)

try:
    while True:
        # Get finger states from MediaPipe
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert the frame to RGB format
        color_image = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_BGR2RGB)

        # Process the frame for holistic pose and hand detection
        results = holistic.process(color_image)

        # Draw landmarks on the frame
        if results.pose_landmarks:
            mp.mp.solutions.drawing_utils.draw_landmarks(
                color_image, results.pose_landmarks, mp.mp.solutions.holistic.POSE_CONNECTIONS
            )
        if results.right_hand_landmarks:
            mp.mp.solutions.drawing_utils.draw_landmarks(
                color_image, results.right_hand_landmarks, mp.mp.solutions.holistic.HAND_CONNECTIONS
            )
            # Detect finger states
            finger_states = mp.get_fingers_states(results.right_hand_landmarks)
            motion_test(finger_states)

        # Convert the image back to BGR for OpenCV display
        color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Pose and Hand Detection', color_image_bgr)

        # Exit on 'a' key press
        if cv2.waitKey(1) & 0xFF == ord('a'):
            break
except KeyboardInterrupt:
    print("CTRL+C detected. Exiting gracefully...")
    shutdownMotor()
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
