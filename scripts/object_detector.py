#!/usr/bin/env python
import rospy
import numpy as np
from drone_autonomous.msg import BlobDetection, ObjectPosition
from geometry_msgs.msg import PointStamped

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
        
        # Parameters
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.target_tolerance = rospy.get_param('~target_tolerance', 20)  # pixels
        
        # Publishers and Subscribers
        self.object_pub = rospy.Publisher('/object_position', ObjectPosition, queue_size=1)
        self.blob_sub = rospy.Subscriber('/blob_detection', BlobDetection, self.blob_callback)
        
        # Target position (center of frame)
        self.target_x = self.frame_width / 2
        self.target_y = self.frame_height / 2
        
    def blob_callback(self, msg):
        obj_msg = ObjectPosition()
        
        if msg.detected:
            # Calculate position differences
            dx = msg.x - self.target_x
            dy = msg.y - self.target_y
            
            obj_msg.detected = True
            obj_msg.x = msg.x
            obj_msg.y = msg.y
            obj_msg.dx = dx
            obj_msg.dy = dy
            obj_msg.centered = (abs(dx) < self.target_tolerance and 
                              abs(dy) < self.target_tolerance)
        else:
            obj_msg.detected = False
            
        self.object_pub.publish(obj_msg)

if __name__ == '__main__':
    od = ObjectDetector()
    rospy.spin()