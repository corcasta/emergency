#!/usr/bin/env python

import rospy
#import tf_conversions
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros

def hard_drive_pose_decorator(buffer, broadcaster):
    tf_buffer = buffer
    br = broadcaster
    def hard_drive_pose_callback(aruco_marker_pose):
        """
        aruco_marker_pose: this transform is with respect base_link
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "marker1"
        t.child_frame_id = "pre_grasp_position"
        t.transform.translation.x = 0.115
        t.transform.translation.y = 0
        t.transform.translation.z = 0.25

        transform_data = tf_buffer.lookup_transform('marker1', 'marker1', rospy.Time())
        print(transform_data)
        t.transform.rotation.x = transform_data.transform.rotation.x
        t.transform.rotation.y = transform_data.transform.rotation.y
        t.transform.rotation.z = transform_data.transform.rotation.z
        t.transform.rotation.w = transform_data.transform.rotation.w
        br.sendTransform(t)


        t2 = TransformStamped()
        t2.header.stamp = rospy.Time.now()
        t2.header.frame_id = "marker1"
        t2.child_frame_id = "max_grasp_position"
        t2.transform.translation.x = 0.115
        t2.transform.translation.y = 0
        t2.transform.translation.z = 0.1
        t2.transform.rotation.x = t.transform.rotation.x
        t2.transform.rotation.y = t.transform.rotation.y
        t2.transform.rotation.z = t.transform.rotation.z
        t2.transform.rotation.w = t.transform.rotation.w    
        br.sendTransform(t2)

        #transform_data = tf_buffer.lookup_transform('marker1', 'marker1', rospy.Time())
        t3 = TransformStamped()
        t3.header.stamp = rospy.Time.now()
        t3.header.frame_id = "marker1"
        t3.child_frame_id = "marker_prime"
        t3.transform.translation.x = 0
        t3.transform.translation.y = 0
        t3.transform.translation.z = 0
        t3.transform.rotation.x = -0.7071068
        t3.transform.rotation.y = 0.7071068
        t3.transform.rotation.z = 0
        t3.transform.rotation.w = 0    
        br.sendTransform(t3)

    return hard_drive_pose_callback


if __name__ == '__main__':
    rospy.init_node('tf_hard_drive_broadcaster')
    print("Funciona")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    hard_drive_pose_callback = hard_drive_pose_decorator(tfBuffer, br)




    rospy.Subscriber("/aruco_single/pose", PoseStamped, hard_drive_pose_callback)
    rospy.spin()
