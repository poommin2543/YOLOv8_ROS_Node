#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RoverControlNode:
    def __init__(self):
        rospy.init_node('rover_control_node')
        self.detection_sub = rospy.Subscriber('/yolov8/detections', String, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def callback(self, data):
        try:
            detections = eval(data.data)  # Be cautious with eval; use safer parsing methods if possible
            twist = Twist()

            if detections:
                target = detections[0]  # Simplified example, assumes the first detection is the target
                
                # Calculate the center of the target box
                target_center_x = (target['x1'] + target['x2']) / 2
                target_center_y = (target['y1'] + target['y2']) / 2

                # Define stopping thresholds (you can adjust these values)
                x_threshold = 50  # Pixels from the center of the frame
                y_threshold = 50  # Pixels from the center of the frame

                # Define frame center
                frame_center_x = 320  # Assuming a 640x480 frame
                frame_center_y = 240  # Assuming a 640x480 frame

                # Calculate the distance from the center of the frame
                distance_x = abs(target_center_x - frame_center_x)
                distance_y = abs(target_center_y - frame_center_y)
                # Stop the car if the target is close enough to the center
                if distance_x < x_threshold and distance_y < y_threshold:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    rospy.loginfo("Target reached, stopping the car.")
                else:
                    # Control logic based on target position
                    if target_center_x < frame_center_x:
                        twist.angular.z = 0.5  # Turn left
                    elif target_center_x > frame_center_x:
                        twist.angular.z = -0.5  # Turn right
                    if target_center_y < frame_center_y:
                        twist.linear.x = 0.5  # Move forward
                    elif target_center_y > frame_center_y:
                        twist.linear.x = -0.5  # Move backward
            else:
                # If no detections, stop the vehicle
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                rospy.loginfo("No detections found, stopping the vehicle.")

            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            rospy.logerr(f"Failed to process detection data: {e}")

if __name__ == '__main__':
    try:
        node = RoverControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
