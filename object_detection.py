import cv2
import time
from ultralytics import YOLO

# Load the YOLO model
model = YOLO('best.pt')

# Initialize the video capture from the webcam
cap = cv2.VideoCapture(0)

# List to keep track of detected items
detected_items = []

# Timer for detecting no objects
start_time = time.time()
detection_timeout = 10  # seconds

# Loop through the video frames
while True:
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame)
        frame_detections = False  # Flag to check if any object is detected in the current frame

        # Check the results for detected objects
        for result in results[0].boxes:
            class_name = model.names[int(result.cls)]
            frame_detections = True

            # Add detected item to the list if not already present
            if class_name not in detected_items:
                detected_items.append(class_name)

        # Reset the timer if objects are detected in the current frame
        if frame_detections:
            start_time = time.time()

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed or if the detection timeout is reached
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        # Check if the detection timeout has been reached
        if time.time() - start_time > detection_timeout:
            print("No object detected for 10 seconds. Exiting...")
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()

# Print the list of detected items
print("Detected items:", detected_items)
