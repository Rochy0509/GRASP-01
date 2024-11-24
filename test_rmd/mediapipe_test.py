import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np

def initialize_pipeline():
    """
    Initializes the RealSense pipeline.
    :return: Initialized RealSense pipeline.
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

def get_fingers_states(hand_landmarks):
    """
    Determines which fingers are up based on the detected landmarks.
    :param hand_landmarks: Mediapipe hand landmarks.
    :return: Dictionary indicating which fingers are up.
    """
    finger_states = {"Thumb": False, "Index": False, "Middle": False, "Ring": False, "Pinky": False}

    # Thumb: Compare x-coordinates for the right hand
    if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x:
        finger_states["Thumb"] = True

    # Other fingers: Compare y-coordinates
    for idx, name in zip([8, 12, 16, 20], ["Index", "Middle", "Ring", "Pinky"]):
        if hand_landmarks.landmark[idx].y < hand_landmarks.landmark[idx - 2].y:
            finger_states[name] = True

    return finger_states

def run_pose_estimation(pipeline, holistic):
    """
    Captures a frame, runs detection, and returns pose and hand landmarks.
    :param pipeline: Initialized RealSense pipeline.
    :param holistic: Initialized MediaPipe holistic detector.
    :return: Detected hand landmarks.
    """
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return None

    # Convert to RGB
    color_image = np.asanyarray(color_frame.get_data())
    rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    # Process the image with MediaPipe
    results = holistic.process(rgb_image)

    # Return landmarks
    return results.right_hand_landmarks
