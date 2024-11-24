import myactuator_rmd_py as rmd
import time
import threading
import matplotlib.pyplot as plt
import mediapipe_test as mp
import cv2
import numpy as np

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
]

motor_ids = [1, 2, 3, 4]
home_positions = [341, 150, 339, 240]

time_data = []
position_data = {motor_id: [] for motor_id in motor_ids}
start_time = time.time()
data_lock = threading.Lock()

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

def motion_test(finger_state):
    """
    Perform motor actions based on the finger state.
    :param finger_state: Dictionary indicating which fingers are up.
    """
    if finger_state["Index"]:
        print("Index finger is up. Moving motors to target positions.")
        motors[0].sendPositionAbsoluteSetpoint(390, 10),
        motors[1].sendPositionAbsoluteSetpoint(100, 10),
        motors[2].sendPositionAbsoluteSetpoint(365, 10),
        motors[3].sendPositionAbsoluteSetpoint(300, 10)
    elif finger_state["Thumb"]:
            print("Index finger is up. Moving motors to target positions.")
            motors[0].sendPositionAbsoluteSetpoint(341, 10),
            motors[1].sendPositionAbsoluteSetpoint(150, 10),
            motors[2].sendPositionAbsoluteSetpoint(339, 10),
            motors[3].sendPositionAbsoluteSetpoint(240, 10)

plot_thread = threading.Thread(target=plot_motor_positions, daemon=True)
plot_thread.start()
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
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
