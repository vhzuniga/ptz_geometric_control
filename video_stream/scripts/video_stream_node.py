#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoStream:
    def __init__(self):
        rospy.init_node("datavideo_node")
        
        self.publisher_setup()

        # Coloca aquí la URL RTSP de tu cámara
        rtsp_url = "rtsp://192.168.0.90:554"

        self.cap = cv2.VideoCapture(rtsp_url)
        self.bridge = CvBridge()

        self.timer = rospy.Timer(rospy.Duration(0.01), self.node_logic)
        rospy.loginfo("Datavideo stream just started")

    def publisher_setup(self):
        self.video_publisher = rospy.Publisher("/datavideo/video", Image, queue_size=1)

    def node_logic(self, timer):
        if not self.cap.isOpened():
            rospy.logwarn("Error: Could not open RTSP stream")
            return
   
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Error: Failed to capture image from stream")
            return
        
        try:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.video_publisher.publish(image_message)
        except Exception as e:
            rospy.logerr(f"cv_bridge conversion error: {e}")

def main():
    video_stream = VideoStream()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        video_stream.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
