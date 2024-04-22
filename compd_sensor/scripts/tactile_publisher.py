# %%
from class_tactile_ import TACTILE
from class_stitch_ import Stitch
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
from class_stereo_ import STEREO
import json
import subprocess
import rospy
from std_msgs.msg import Float32,Float32MultiArray,Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from itf.msg import array
import math
import time

# %%
def publisher1(num1):
    pub1 = rospy.Publisher('tactile_msg1', Float32, queue_size=10)
    rospy.init_node('tactile1', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        msg1 = Float32()
        msg1.data = abs(np.mean(num1))
        pub1.publish(msg1)
        rate.sleep()

def publisher2(num2):
    pub2 = rospy.Publisher('tactile_msg2', Float32, queue_size=10)
    rospy.init_node('tactile2', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        msg2 = Float32()
        msg2.data = abs(np.mean(num2))
        pub2.publish(msg2)
        rate.sleep()


stitch = Stitch()

touch = TACTILE(param_file="/home/lifan/itf_ws/src/itf/scripts/config/tactile_config/blob.json",threshold=45)


# %%
reference_path = "/home/lifan/itf_ws/src/itf/scripts/config/tactile_config/reference.png"
refimage = cv2.imread(reference_path)

# %%
def tactile_function(original_image,refimage):
    try:
        stitch.get_coordinates(original_image)
        # original_pic = stitch.original_pic()
        stitch.divide()
        stitched_image = stitch.all_stitch()
        stitched_image = stitched_image[:,20:-30]
        blur_image = cv2.GaussianBlur(stitched_image, (5,5),0)
        hsv_image = cv2.cvtColor(blur_image,cv2.COLOR_BGR2HSV)
        hsv_image = stitch.classify(hsv_image,160,250) #160,250
        hsved_image = cv2.cvtColor(hsv_image,cv2.COLOR_HSV2RGB)
        blob_image,_,_ = stitch.draw_black(hsved_image,param_file = '/home/lifan/itf_ws/src/itf/scripts/config/tactile_config/draw_black.json')    
        touch.find_ref(refimage)
        recent_loc = touch.get_disp(blob_image)
    except:
        recent_loc = np.zeros(8,2)

    return recent_loc

# %%1

def callback1(data):
    # rospy.loginfo("callback1")
    ans_t1 = tactile_function(data.data)
    publisher1(ans_t1)
    
def callback2(data):
    # rospy.loginfo("callback2")
    ans_t2 = tactile_function(data.data)
    publisher2(ans_t2)

rospy.init_node("tactile_node",anonymous=True)
rospy.Subscriber("sensor1",Image,callback1)
rospy.Subscriber("sensor2",Image,callback2)

