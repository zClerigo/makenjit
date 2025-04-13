import cv2
import numpy as np
import torch
import mediapipe as mp
from PIL import Image

def display_webcam():
    # Initialize the webcam (0 is usually the default camera)
    cap = cv2.VideoCapture(0)
    
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    
    # Load YOLOv5 model from PyTorch Hub
    print("Loading YOLOv5 model...")
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
    
    # Initialize MediaPipe Hands
    print("Initializing MediaPipe Hands...")
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    
    print("Webcam started with YOLOv5 person detection and finger tracking. Press 'q' to quit.")
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # If frame is read correctly, ret is True
        if not ret:
            print("Error: Can't receive frame. Exiting...")
            break
        
        # YOLOv5 detection
        results = model(frame)
        
        # Process detection results - only count people (class 0)
        person_count = (results.pred[0][:, -1] == 0).sum().item()
        
        # Draw bounding boxes and labels for people only
        for *xyxy, conf, cls in results.pred[0]:
            cls = int(cls)
            
            if cls == 0:  # Person
                x1, y1, x2, y2 = map(int, xyxy)
                color = (255, 0, 0)  # Red for person
                label = f"Person: {conf:.2f}"
                
                # Draw rectangle and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Display count on the frame
        cv2.putText(frame, f"People: {person_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Hand and finger detection with MediaPipe
        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results_hands = hands.process(rgb_frame)
        
        # Draw hand landmarks
        if results_hands.multi_hand_landmarks:
            for hand_landmarks in results_hands.multi_hand_landmarks:
                # Draw the hand landmarks
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # Count extended fingers
                extended_fingers = count_extended_fingers(hand_landmarks)
                
                # Get hand position for text placement
                hand_x = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * frame.shape[1])
                hand_y = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * frame.shape[0])
                
                # Display finger count
                cv2.putText(frame, f"Fingers: {extended_fingers}", 
                           (hand_x, hand_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display the resulting frame
        cv2.imshow('Webcam with YOLOv5 Person Detection and Finger Tracking', frame)
        
        # Press 'q' to exit
        if cv2.waitKey(1) == ord('q'):
            break
    
    # Release the webcam and close all windows
    hands.close()
    cap.release()
    cv2.destroyAllWindows()

def count_extended_fingers(hand_landmarks):
    """Count the number of extended fingers in a hand."""
    # Get fingertip and pip (proximal interphalangeal joint) landmarks
    fingertips = [
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP],
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP],
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP],
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.RING_FINGER_TIP],
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.PINKY_TIP]
    ]
    
    finger_pips = [
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.THUMB_IP],  # For thumb, use IP instead of PIP
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP],
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP],
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.RING_FINGER_PIP],
        hand_landmarks.landmark[mp.solutions.hands.HandLandmark.PINKY_PIP]
    ]
    
    # Get wrist landmark for thumb detection
    wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
    
    # Count extended fingers
    extended_count = 0
    
    # Special case for thumb (compare x-coordinate relative to wrist)
    if fingertips[0].x < wrist.x:  # For right hand
        if fingertips[0].x < finger_pips[0].x:
            extended_count += 1
    else:  # For left hand
        if fingertips[0].x > finger_pips[0].x:
            extended_count += 1
    
    # For other fingers, check if fingertip is higher than PIP joint
    for i in range(1, 5):
        if fingertips[i].y < finger_pips[i].y:
            extended_count += 1
    
    return extended_count

if __name__ == "__main__":
    display_webcam()