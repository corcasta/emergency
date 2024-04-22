import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# Publisher
def line_detector_publisher():
    pub1 = rospy.Publisher('line_detector', Bool, queue_size=10)
    rospy.init_node('line_node', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        msg1 = Bool()
        msg1.data = True
        # msg1.data = abs(np.mean(num1))
        pub1.publish(msg1)
        rate.sleep()

# The Function for detecting the line
def line_detector():
    return True

# Separating the image into 6 windows
def separate_function(image):
    window_array = []
    
    m = [[500,500,700,700],
         [500,500,700,700],
         [500,500,700,700],
         [500,500,700,700],
         [500,500,700,700],
         [500,500,700,700]]
    for i in range(6):

        # a,b,c,d = 500,700,500,700
        window = image[m[i][0]:m[i][1],m[i][2]:m[i][3]]
        window_array.append(window)

#callback function
def callback1(data):
    window_array = separate_function(data.data)
    if line_detector is True:
        line_detector_publisher()
        
def callback2(data):
    window_array = separate_function(data.data)
    if line_detector is True:
        line_detector_publisher()

rospy.init_node("window_node",anonymous=True)

rospy.Subscriber("sensor1",Image,callback1)
rospy.Subscriber("sensor2",Image,callback2)