#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_autonomous.msg import BlobDetection

class CameraProcessor:
    def __init__(self):
        rospy.init_node('camera_processor', anonymous=True)
        self.bridge = CvBridge()
        
        # Parameters
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.hsv_min = rospy.get_param('~hsv_min', [77, 40, 0])
        self.hsv_max = rospy.get_param('~hsv_max', [101, 255, 255])
        
        # Publishers and Subscribers
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        self.blob_pub = rospy.Publisher('/blob_detection', BlobDetection, queue_size=1)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        
    def apply_search_window(self, image, window=[0.0, 0.0, 1.0, 1.0]):
        rows, cols = image.shape[:2]
        x_min, y_min = int(cols * window[0]), int(rows * window[1])
        x_max, y_max = int(cols * window[2]), int(rows * window[3])
        mask = np.zeros_like(image)
        mask[y_min:y_max, x_min:x_max] = image[y_min:y_max, x_min:x_max]
        return mask
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Blob detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, tuple(self.hsv_min), tuple(self.hsv_max))
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.erode(mask, None, iterations=2)
            mask = self.apply_search_window(mask)
            
            # Find contours
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            blob_msg = BlobDetection()
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                blob_msg.detected = True
                blob_msg.x = x
                blob_msg.y = y
                blob_msg.radius = radius
                
                # Draw detection on image
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), -1)
            
            # Publish results
            self.blob_pub.publish(blob_msg)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")

if __name__ == '__main__':
    cp = CameraProcessor()
    rospy.spin()