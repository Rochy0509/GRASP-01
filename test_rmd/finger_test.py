import pyrealsense2 as rs
import mediapipe as mp
import cv2
import numpy as np

def get_finger_states(hand_landmarks):
    """
    Determines which fingers are up based on the hand landmarks.
    :param hand_landmarks: List of landmarks for the detected hand.
    :return: A dictionary indicating which fingers are up.
    """
    finger_states = {
        "Thumb": False,
        "Index": False,
        "Middle": False,
        "Ring": False,
        "Pinky": False,
        "FullyClosed": False
    }

    # Thumb tip and thumb MCP landmarks
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]

    # Index finger landmarks
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_dip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP]

    # Middle finger landmarks
    middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_dip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP]

    # Ring finger landmarks
    ring_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_dip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP]

    # Pinky finger landmarks
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_dip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP]

    # Check if each finger is up
    finger_states["Thumb"] = thumb_tip.y < thumb_ip.y  # Thumb is up if tip is above IP
    finger_states["Index"] = index_tip.y < index_dip.y  # Index is up if tip is above DIP
    finger_states["Middle"] = middle_tip.y < middle_dip.y  # Middle is up if tip is above DIP
    finger_states["Ring"] = ring_tip.y < ring_dip.y  # Ring is up if tip is above DIP
    finger_states["Pinky"] = pinky_tip.y < pinky_dip.y  # Pinky is up if tip is above DIP

    # Fully closed hand check (all fingers down)
    if not any(finger_states.values()):
        finger_states["FullyClosed"] = True

    return finger_states


# Initialize MediaPipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Get frames from RealSense camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert frame to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Flip the frame horizontally for natural mirroring
        frame = cv2.flip(frame, 1)

        # Convert the frame to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame for hand landmarks
        results = hands.process(rgb_frame)

        # Draw hand landmarks and analyze finger states
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS
                )

                # Analyze finger states
                finger_states = get_finger_states(hand_landmarks)
                print(f"Finger States: {finger_states}")

                # Display the recognized gesture
                gesture = ""
                if finger_states["FullyClosed"]:
                    gesture = "Fully Closed Hand - Home Position"
                elif finger_states["Thumb"] and not finger_states["Index"] and not finger_states["Middle"]:
                    gesture = "Thumb Up (Fully Closed) - Picking Pose"
                elif finger_states["Index"] and finger_states["Middle"] and not finger_states["Thumb"]:
                    gesture = "Index + Middle Up (Thumb Down) - Dropping Pose"
                elif finger_states["Index"] and finger_states["Thumb"] and not finger_states["Middle"]:
                    gesture = "Index + Thumb Up - Return to Picking Pose"
                else:
                    gesture = "Unknown Gesture"

                # Display the gesture on the frame
                cv2.putText(frame, gesture, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Display the frame
        cv2.imshow('Finger State Detection', frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
