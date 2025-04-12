import cv2
import numpy as np
import torch
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
    
    print("Webcam started with YOLOv5 person detection. Press 'q' to quit.")
    
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
        
        # Display the resulting frame
        cv2.imshow('Webcam with YOLOv5 Person Detection', frame)
        
        # Press 'q' to exit
        if cv2.waitKey(1) == ord('q'):
            break
    
    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    display_webcam()