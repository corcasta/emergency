#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def image_publisher(frame1,frame2):
    rospy.init_node('image_publisher',anonymous=True)
    image_pub1 = rospy.Publisher('sensor1',Image,queue_size=10)
    image_pub2 = rospy.Publisher('sensor2',Image,queue_size=10)
    rate = rospy.Rate(1)

    bridge = CvBridge()

    while not rospy.is_shutdown():

        ros_image1 = bridge.cv2_to_imgmsg(frame1,encoding="bgr8")
        ros_image2 = bridge.cv2_to_imgmsg(frame2,encoding="bgr8")

        image_pub1.publish(ros_image1)
        image_pub2.publish(ros_image2)
        rate.sleep()

# def image_publisher(frame1):
#     rospy.init_node('image_publisher',anonymous=True)
#     image_pub1 = rospy.Publisher('sensor1',Image,queue_size=10)
#     # image_pub2 = rospy.Publisher('sensor2',Image,queue_size=10)
#     rate = rospy.Rate(1)

#     bridge = CvBridge()

#     while not rospy.is_shutdown():

#         ros_image1 = bridge.cv2_to_imgmsg(frame1,encoding="bgr8")
#         # ros_image2 = bridge.cv2_to_imgmsg(frame2,encoding="bgr8")

#         image_pub1.publish(ros_image1)
#         # image_pub2.publish(ros_image2)
#         rate.sleep()


cap0 = cv2.VideoCapture(3)
cap0.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cap1 = cv2.VideoCapture(8) # change the id
cap1.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


while 1:
    print("published msg")
    _,frame1 = cap0.read()
    _,frame2 = cap1.read()
    
    image_publisher(frame1,frame2)