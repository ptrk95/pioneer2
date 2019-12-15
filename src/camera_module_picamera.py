#!/usr/bin/env python

import cv2 

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import picamera
import picamera.array

URL = "http://192.168.0.5:8080/video"

def Camera():
    cv_bridge = CvBridge()
    pub_rate = rospy.get_param("camera_module/publish_rate", default=30)
    height = rospy.get_param("camera_module/height", default=720)
    width = rospy.get_param("camera_module/width", default=1280)

    # publish data to topic
    pub_img1 = rospy.Publisher('camera_module/video_stream', Image, queue_size=1)


    rospy.init_node('camera_module', anonymous=True)
    rate = rospy.Rate(pub_rate) # 10hz

    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.resolution = (width, height)
            camera.framerate = pub_rate
            while not rospy.is_shutdown():
                camera.capture(stream, 'bgr', use_video_port=True)
                frame = stream.array

                try:
                    pub_img1.publish(cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
                except CvBridgeError as e:
                    print(e)
                    rospy.logerr(e)
                    
                stream.seek(0)
                stream.truncate()
                rate.sleep()


if __name__ == '__main__':
    try:
        Camera()
    except rospy.ROSInterruptException:
        pass

