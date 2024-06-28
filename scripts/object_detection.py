#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import time

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node')
        self.publisher = rospy.Publisher('/detected_objects', String, queue_size=10)
        self.detection_complete_pub = rospy.Publisher('/detection_complete', String, queue_size=10)

        # Load YOLO model
        self.model = YOLO('home/mustar/catkin_ws/src/robot_cooking_assistant/scripts/best.pt')
        self.cap = cv2.VideoCapture('/dev/video2') #Camera name might change depending on the device

        self.detected_items = []
        self.start_time = time.time()
        self.detection_timeout = 10
        rospy.loginfo("Object Detection Node Initialized")

    def run(self):
        rate = rospy.Rate(17)
        while not rospy.is_shutdown():
            success, frame = self.cap.read()

            if success:
                # Run YOLOv8 inference on the frame
                results = self.model(frame)
                frame_detections = False

                # Check the results for detected objects
                for result in results[0].boxes:
                    class_name = self.model.names[int(result.cls)]
                    frame_detections = True

                    # Add detected item to the list if not already present
                    if class_name not in self.detected_items:
                        self.detected_items.append(class_name)
                        rospy.loginfo(f"Detected new object: {class_name}")

                # Reset the timer if objects are detected in the current frame
                if frame_detections:
                    self.start_time = time.time()

                detected_items_str = ','.join(self.detected_items)
                self.publisher.publish(detected_items_str)
                rospy.loginfo(f"Published detected objects: {detected_items_str}")

                # Visualize the results on the frame
                annotated_frame = results[0].plot()

                # Display the annotated frame
                cv2.imshow("YOLOv8 Inference", annotated_frame)

                # Break the loop if 'q' is pressed or if the detection timeout is reached
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                # Check if the detection timeout has been reached
                if time.time() - self.start_time > self.detection_timeout:
                    rospy.loginfo("No object detected for 10 seconds. Exiting...")
                    break
            else:
                # Break the loop if the end of the video is reached
                break

            rate.sleep()

        # Signal completion
        self.detection_complete_pub.publish("Object detection complete")
        rospy.loginfo("Detection completion message published")

        # Release the camera and close OpenCV windows
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
