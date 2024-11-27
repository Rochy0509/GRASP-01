import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np

# Initialize MediaPipe Holistic
mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils

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
    finger_states = {"Thumb": False, "Index": False, "Middle": False, "Ring": False, "Pinky": False, "FullyClosed": False}

    # Thumb detection for the right hand
    if hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x:
        finger_states["Thumb"] = True

    # Other fingers detection
    for idx, name in zip([8, 12, 16, 20], ["Index", "Middle", "Ring", "Pinky"]):
        if hand_landmarks.landmark[idx].y < hand_landmarks.landmark[idx - 2].y:
            finger_states[name] = True

    # Fully closed hand: all fingers are down
    finger_states["FullyClosed"] = not any(finger_states.values())
    return finger_states


def run_pose_estimation(pipeline, holistic):
    """
    Captures a frame, runs detection, and returns pose and hand landmarks.
    :param pipeline: Initialized RealSense pipeline.
    :param holistic: Initialized MediaPipe holistic detector.
    :return: Processed MediaPipe results.
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
    if results.right_hand_landmarks:
        finger_states = get_fingers_states(results.right_hand_landmarks)
        return color_image, finger_states
    else:
        return color_image, None


def main():
    # Initialize RealSense pipeline and MediaPipe Holistic
    pipeline = initialize_pipeline()
    with mp_holistic.Holistic(static_image_mode=False, model_complexity=1, enable_segmentation=False, min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
        try:
            while True:
                # Get the frame and MediaPipe results
                frame, results = run_pose_estimation(pipeline, holistic)
                if frame is None or results is None:
                    continue

                # Draw landmarks for the right arm and hand
                annotated_frame = frame.copy()
                if results.right_hand_landmarks:
                    mp_drawing.draw_landmarks(annotated_frame, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
                    finger_states = get_fingers_states(results.right_hand_landmarks)
                    print("Finger States:", finger_states)

                if results.pose_landmarks:
                    # Draw right arm connections (shoulder, elbow, wrist)
                    mp_drawing.draw_landmarks(
                        annotated_frame,
                        results.pose_landmarks,
                        mp_holistic.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2, circle_radius=2),
                        connection_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
                    )

                # Show the annotated frame
                cv2.imshow("Right Hand and Arm Tracking", annotated_frame)

                # Exit on 'q' key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            # Cleanup
            pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
