import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
import pyrealsense2 as rs
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from scipy.spatial.transform import Rotation


class D405(object):
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.pipeline.start(self.config)
    
    def start(self):
        self.frames = self.pipeline.wait_for_frames()
        
    def get_depth(self,x,y):
        self.depth = self.frames.get_depth_frame()
        d = self.depth.get_distance(x,y)
        return d
    
    def get_frame(self):
        self.color_frame = self.frames.get_color_frame()
        self.color_image = np.asanyarray(self.color_frame.get_data())
        return self.color_image

    def shut_down(self):
        self.pipeline.stop()

    def show(self):
        # while 1:
        self.start()
        frame = self.get_frame()
        cv2.imshow("D405",frame)
        cv2.waitKey(0)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break


class Aruco(object):
    def detect_aruco(self, frame):
        self.frame = frame
        # Load the ArUco dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        
        # Convert the frame to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        parameters = aruco.DetectorParameters()

        self.corners, self.ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        return self.corners, self.ids
        
    def centralize(self, corners):
        
        length = len(corners)
        cx_list = []
        cy_list = []

        for k in range(length):
            area = 0
            cx,cy = 0,0
            x = []
            y = []
            corner = corners[k][0]
            # print(corner[0][0])

            x = [corner[0][0],corner[1][0],corner[2][0],corner[3][0]]
            y = [corner[0][1],corner[1][1],corner[2][1],corner[3][1]]
            
            n = len(x)

            area = 0
            for i in range(n):
                j = (i + 1) % n
                area += (x[i] * y[j]) - (x[j] * y[i])
            area = abs(area)/2
            
            for i in range(n):
                j = (i + 1) % n
                cx += 1/6/area * (x[i] + x[j]) * ((x[i] * y[j]) - (x[j] * y[i]))
                cy += 1/6/area * (y[i] + y[j]) * ((x[i] * y[j]) - (x[j] * y[i]))
            cx = np.int64(cx)
            cy = np.int64(cy)

            cx_list.append(cx)
            cy_list.append(cy)

        return cx_list,cy_list

    def single_centralize(self, corners):
        
        length = len(corners)
        # cx_list = []
        # cy_list = []

        for k in range(length):
            area = 0
            cx,cy = 0,0
            x = []
            y = []
            corner = corners[k][0]
            # print(corner[0][0])

            x = [corner[0][0],corner[1][0],corner[2][0],corner[3][0]]
            y = [corner[0][1],corner[1][1],corner[2][1],corner[3][1]]
            
            n = len(x)

            area = 0
            for i in range(n):
                j = (i + 1) % n
                area += (x[i] * y[j]) - (x[j] * y[i])
            area = abs(area)/2
            
            for i in range(n):
                j = (i + 1) % n
                cx += 1/6/area * (x[i] + x[j]) * ((x[i] * y[j]) - (x[j] * y[i]))
                cy += 1/6/area * (y[i] + y[j]) * ((x[i] * y[j]) - (x[j] * y[i]))
            cx = np.int64(cx)
            cy = np.int64(cy)

            # cx_list.append(cx)
            # cy_list.append(cy)

        return cx,cy
    
    ''''''
    def rotation_matrix_to_euler(self,matrix):
        r = Rotation.from_matrix(matrix)
        euler = r.as_euler('xyz')
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    def estimate_pose(self,calibration_matrix_path,distortion_coefficients_path):
        k = np.load(calibration_matrix_path)
        d = np.load(distortion_coefficients_path)
        self.roll,self.pitch,self.yaw = None,None,None
        if len(self.corners) > 0:
            for i in range(0,len(self.ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(self.corners[i], 0.02, k,d)
                rotation_matrix = cv2.Rodrigues(rvec)[0]
                self.roll, self.pitch, self.yaw = self.rotation_matrix_to_euler(rotation_matrix)
                self.roll = math.degrees(self.roll)
                self.pitch = math.degrees(self.pitch)
                self.yaw = math.degrees(self.yaw)
    ''''''

    def vector_average(self, v):
        sum_array = np.sum(v,axis=0)
        average_array = sum_array / len(v)
        result = average_array.tolist()    
        return result

    def dot(self, list1, list2):
        return list1[0]*list2[0]+list1[1]*list2[1]

    def angle(self, list1, list2):
        return math.asin((list1[0]*list2[0]+list1[1]*list2[1])/(np.linalg.norm(list1)*np.linalg.norm(list2)))

    def aruco_computation(self, corner_list):
        length = len(corner_list)
        v1_list = []
        v2_list = []
        v3_list = []
        v4_list = []
        angle_list = []
        # print(shape)
        # print(corner_list[0][0][0][0][0]) #corner_list[the window idx][the number of recognized markers][0][the row][x or y] 
        # rospy.sleep(100)
        for i in range(length):
            v1 = corner_list[i][0][0][1] - corner_list[i][0][0][0]
            v2 = corner_list[i][0][0][2] - corner_list[i][0][0][1]
            v3 = corner_list[i][0][0][3] - corner_list[i][0][0][2]
            v4 = corner_list[i][0][0][0] - corner_list[i][0][0][3]
            v1_list.append(v1)
            v2_list.append(v2)
            v3_list.append(v3)
            v4_list.append(v4)
        '''average result'''
        v1_result = self.vector_average(v1_list)
        v2_result = self.vector_average(v2_list)
        v3_result = self.vector_average(v3_list)
        v4_result = self.vector_average(v4_list)
        '''unitization'''
        v1_unit = v1_result / np.linalg.norm(v1_result)
        v2_unit = v2_result / np.linalg.norm(v2_result)
        v3_unit = v3_result / np.linalg.norm(v3_result)
        v4_unit = v4_result / np.linalg.norm(v4_result)
        '''a dot b = 0'''
        vert1 = self.dot(v1_unit, v2_unit) # a dot b is 0 for vertical 
        vert2 = self.dot(v2_unit, v3_unit)
        vert3 = self.dot(v3_unit, v4_unit)
        vert4 = self.dot(v4_unit, v1_unit)
        '''no rotation'''
        v_h = [0,1]
        v_v = [1,0]

        a1 = math.degrees(self.angle(v1_unit, v_h))
        a2 = math.degrees(self.angle(v2_unit, v_v))
        a3 = math.degrees(self.angle(v3_unit, v_h))
        a4 = math.degrees(self.angle(v4_unit, v_v))

        angle_list.append(a1)
        angle_list.append(a2)
        angle_list.append(a3)
        angle_list.append(a4)

        # angle_list.append(round(vert1,3))
        # angle_list.append(round(vert2,3))
        # angle_list.append(round(vert3,3))
        # angle_list.append(round(vert4,3))

        return angle_list

    def draw_center(self,cx_list,cy_list):
        radius = 5  # Radius of the point
        color = (0, 0, 255)  # Red color (BGR format)
        thickness = -1  # Fill the point
        for i in range(len(cx_list)):

            cv2.circle(self.frame, (cx_list[i], cy_list[i]), radius, color, thickness)

        cv2.imshow('CENTER', self.frame)
        cv2.waitKey(0)

# if __name__ == "__main__":
#     a = Aruco()
#     img = cv2.imread("/home/robot/Documents/itf_project/2024-04-03-165535.jpg")

#     a.detect_aruco(img)
#     cx_list,cy_list = a.centralize(a.corners)
    
#     a.draw_center(cx_list,cy_list)