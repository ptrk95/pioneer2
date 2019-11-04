#!/usr/bin/env python

import cv2 
import time
from imutils.video import VideoStream
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imutils import resize

URL = "http://192.168.0.5:8080/video"

def Camera():
    cv_bridge = CvBridge()
    pub_rate = rospy.get_param("camera_module/publish_rate", default=30)
    height = rospy.get_param("camera_module/height", default=720)

    # publish data to topic
    pub_img1 = rospy.Publisher('camera_module/video_stream', Image, queue_size=1)


    rospy.init_node('camera_module', anonymous=True)
    rate = rospy.Rate(pub_rate) # 10hz

    if(rospy.get_param("/image_buffer/usePiCamera", default=True)==False):
        vs = VideoStream(src=URL).start()
    else:
        vs = VideoStream(usePiCamera=True, resolution=(640,480)).start()
    # wait for camera 2 secs
    time.sleep(1.0)    
        
    while not rospy.is_shutdown():
        frame = vs.read()
        

        try:
            pub_img1.publish(cv_bridge.cv2_to_imgmsg(frame, "bgr8"))


        except CvBridgeError as e:
			print(e)
			rospy.logerr(e)

        rate.sleep()
    vs.stop()

if __name__ == '__main__':
    try:
        Camera()
    except rospy.ROSInterruptException:
        pass

