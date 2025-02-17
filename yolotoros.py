#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YOLOv8Node:
    def __init__(self):
        rospy.init_node('yolov8_node')
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.detection_pub = rospy.Publisher('/yolov8/detections', String, queue_size=10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Ensure the path to your model weights
        self.model.to('cuda')  # Force the model to run on the GPU

        # Open the webcam (0 for default camera, or adjust as needed)
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open video stream.")
            exit()
            
        # Get the width and height of the frame
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rospy.loginfo(f"Camera Resolution: {self.width}x{self.height}")

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to grab frame")
                break

            # Perform inference
            results = self.model(frame)

            # Extract boxes, confidences, and class IDs from results
            detections = []
            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy().astype(int)

                for box, confidence, class_id in zip(boxes, confidences, class_ids):
                    x1, y1, x2, y2 = box
                    if class_id == 0:
                        if ((int(x1) < int(y1)) and (int(x2)-int(x1)) > (int(y2)-int(y1))):
                            label = f'{self.model.names[class_id]}: {confidence:.2f}'
                            color = (0, 255, 0)  # Green color for bounding box
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            detections.append({'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2), 'confidence': confidence, 'class_id': class_id})

            # Publish the detection results
            self.detection_pub.publish(str(detections))

            # Convert frame to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(ros_image)

            # Display the resulting frame
            cv2.imshow('YOLOv8 Detection', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the capture and destroy all windows
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = YOLOv8Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
