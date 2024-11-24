import myactuator_rmd_py as rmd
import time
import pyrealsense2 as rs
import mediapipe as mp
import numpy as np
import cv2

# Initialize CAN driver and motor interfaces
driver = rmd.CanDriver("can0")
motors = [
    rmd.ActuatorInterface(driver, 4),  # Roll (Motor 1)
    rmd.ActuatorInterface(driver, 5),  # Yaw (Motor 2)
    rmd.ActuatorInterface(driver, 6),  # Roll again (Motor 3)
]

# Initialize RealSense camera with full resolution
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
pipeline.start(config)

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Threshold for movement
THRESHOLD = 10.0

# Track previous positions to avoid frequent small adjustments
previous_positions = {"roll_1": 0, "yaw": 0, "roll_2": 0}

# Function to constrain angle within 0 to 90 degrees
def constrain_angle(angle, min_angle=0, max_angle=90):
    return max(min(angle, max_angle), min_angle)

def homing_motor(motor):
    tolerance = 0.1
    home_position = 0

    while True:
        current_pos_encoder = motor.getMultiTurnAngle()
        print(f"Current Position {motor.getCanId()}: {current_pos_encoder}") 

        if abs(current_pos_encoder - home_position) <= tolerance:
            print(f'Motor {motor.getCanId()} is at home position.')
            break
        
        motor.sendPositionAbsoluteSetpoint(0, 100)
        time.sleep(0.1)

    motor.setEncoderZero(1)

def home_all_motors():
    for motor in motors:
        homing_motor(motor)
    print("All motors homed and zeroed")

home_all_motors()

def calculate_right_shoulder_angles(landmarks):
    shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
    elbow = landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value]
    hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value]
    
    shoulder_point = np.array([shoulder.x, shoulder.y, shoulder.z])
    elbow_point = np.array([elbow.x, elbow.y, elbow.z])
    hip_point = np.array([hip.x, hip.y, hip.z])

    # Motor 1: Roll (rotation along shoulder's axis)
    shoulder_to_elbow = elbow_point - shoulder_point
    roll_1 = np.arctan2(shoulder_to_elbow[1], shoulder_to_elbow[0]) * 180 / np.pi  # Rotation around the shoulder

    # Motor 2: Yaw (arm lifted from body outward)
    shoulder_to_hip = hip_point - shoulder_point
    yaw = np.arctan2(shoulder_to_elbow[1], shoulder_to_hip[1]) * 180 / np.pi  # Angle for moving arm laterally

    # Motor 3: Roll (rotation along the arm itself)
    roll_2 = np.arctan2(shoulder_to_elbow[2], shoulder_to_elbow[0]) * 180 / np.pi  # Rotation along the arm's axis

    return roll_1, yaw, roll_2

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert frame to numpy array
        frame_data = np.asanyarray(color_frame.get_data())

        # Process frame with MediaPipe Pose
        results = pose.process(frame_data)

        if results.pose_landmarks:
            # Draw landmarks on the frame
            mp.solutions.drawing_utils.draw_landmarks(
                frame_data, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # Calculate right shoulder angles based on new definitions
            roll_1, yaw, roll_2 = calculate_right_shoulder_angles(results.pose_landmarks.landmark)

            # Constrain angles within 0 to 90 degrees
            roll_1 = constrain_angle(roll_1)
            yaw = constrain_angle(yaw)
            roll_2 = constrain_angle(roll_2)

            # Check if angle differences exceed threshold before updating motors
            if abs(roll_1 - previous_positions["roll_1"]) > THRESHOLD:
                motors[0].sendPositionAbsoluteSetpoint(roll_1, 100.0)
                previous_positions["roll_1"] = roll_1

            if abs(yaw - previous_positions["yaw"]) > THRESHOLD:
                motors[1].sendPositionAbsoluteSetpoint(yaw, 100.0)
                previous_positions["yaw"] = yaw

            if abs(roll_2 - previous_positions["roll_2"]) > THRESHOLD:
                motors[2].sendPositionAbsoluteSetpoint(roll_2, 100.0)
                previous_positions["roll_2"] = roll_2

            # Print angle information on the frame
            cv2.putText(frame_data, f"Roll 1: {roll_1:.2f} degrees", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame_data, f"Yaw: {yaw:.2f} degrees", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame_data, f"Roll 2: {roll_2:.2f} degrees", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Display camera feed with landmarks and angles
        cv2.imshow("RealSense Camera Feed with Landmarks and Angles", frame_data)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting program.")
finally:
    pipeline.stop()
    pose.close()
    cv2.destroyAllWindows()
