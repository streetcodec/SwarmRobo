#! /usr/bin/env python3

"""
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
"""
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
import time
import numpy as np
import math
# Import the required modules
##############################################################

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__("ar_uco_detector")
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/image_rect_color",  # Replace with your camera topic
            self.image_callback,
            10,
        )
        self.publisher_1 = self.create_publisher(Pose2D, "/detected_aruco_1", 10)
        self.publisherAngleAruco1 = self.create_publisher(Float32, "/AngleAruco1", 10)
        self.publisher_2 = self.create_publisher(Pose2D, "/detected_aruco_2", 10)
        self.publisher_3 = self.create_publisher(Pose2D, "/detected_aruco_3", 10)
    def image_callback(self, msg):
        # # convert ROS image to opencv image
        # # Detect Aruco marker
        # # Publish the bot coordinates to the topic  /detected_aruco
        # # ============================================= 
        

        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            parameters =  cv2.aruco.DetectorParameters()
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, aruco_dict, parameters=parameters
            )
            # print("##########REJECTED#######")
            # print(_)
            # print("##########REJECTED#########")
            idx4 = -1
            idx8 = -1
            idx10 = -1
            idx12 = -1
            for index, value in enumerate(ids):
                if value == 4:
                    # print(f"Init Element 4 found at index {index}")
                    idx4 = index
                if value == 8:
                    # print(f"Init Element 8 found at index {index}")
                    idx8 = index
                if value == 10:
                    # print(f"Init Element 10 found at index {index}")
                    idx10 = index
                if value == 12:
                    # print(f"Init Element 12 found at index {index}")
                    idx12 = index
            # for i in range(4):
            #     if idx4 != -1 :
            #         # print("Original id 4 aruco points : ")
            #         # print(corners[idx4][0][i][0])
            #         # print(corners[idx4][0][i][1])
                    
            # for i in range(4):
            #     if idx8 != -1 :
            #         # print("Original id 8 aruco points : ")
            #         # print(corners[idx8][0][i][0])
            #         # print(corners[idx8][0][i][1])
            # for i in range(4):
            #     if idx10 != -1 :
            #         # print("Original id 10 aruco points : ")
            #         # print(corners[idx10][0][i][0])
            #         # print(corners[idx10][0][i][1])
            # for i in range(4):
            #     if idx12 != -1 :
            #         # print("Original id 12 aruco points : ")
            #         # print(corners[idx12][0][i][0])
            #         # print(corners[idx12][0][i][1])
            # roi_corners = np.array([(84, 10), (461, 24), (67, 466), (466, 462)])
                    
            roi_corners = np.array([(corners[idx10][0][1][0],corners[idx10][0][1][1]),(corners[idx12][0][2][0],corners[idx12][0][2][1]), (corners[idx8][0][0][0],corners[idx8][0][0][1]), (corners[idx4][0][3][0],corners[idx4][0][3][1])])        
            print(roi_corners)
            print("***********************************************************************************************************************")
            output_size = (500, 500)  # Size of the output rectangle
            
            output_corners = np.array([(0, 0), (499,0), (0,499), (499,499)])

            transform_matrix, _ = cv2.findHomography(roi_corners, output_corners)

            # Perform the perspective transformation
            cv_image2 = cv2.warpPerspective(cv_image, transform_matrix, output_size)
            # Detect ArUco markers
            #cv2.imshow('Transformed Image', cv_image)
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image2, aruco_dict, parameters=parameters
            )
            idx1 = 0
            idx2 = 0
            idx3 = 0
            
            f1 = False
            for index, value in enumerate(ids):
                if value == 1:
                    print(f"Element 1 found at index {index}")
                    idx1 = index
                    f1 = True
                if value == 2:
                    # print(f"Element 2 found at index {index}")
                    idx2 = index
                if value == 3:
                    # print(f"Element 3 found at index {index}")
                    idx3 = index
                if value == 4:
                    # print(f"Element 4 found at index {index}")
                    idx4 = index
                if value == 8:
                    # print(f"Element 8 found at index {index}")
                    idx8 = index
                if value == 10:
                    # print(f"Element 10 found at index {index}")
                    idx10 = index
                if value == 12:
                    # print(f"Element 12 found at index {index}")
                    idx12 = index
            
                    
            for i in range(len(ids)):
                    marker_id = ids[i][0]
                    c = corners[i][0][0]
                    x, y = int(c[0]), int(c[1])
                    text = f'id:{marker_id}'
                    cv2.putText(cv_image2, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0,0), 2)


            file_path = "img.png"
            file_path2 = "imgt.png"


            cv2.imwrite(file_path, cv_image)
            cv2.imwrite(file_path2, cv_image2)
            print(corners)
            print("***********")

            print(ids)

            x1 = 0
            y1 = 0
            x2 = 0
            y2 = 0
            x3 = 0
            y3 = 0
            Aruco1Slope = (corners[idx1][0][1][1] - corners[idx1][0][2][1])/(corners[idx1][0][1][0] - corners[idx1][0][2][0]) 
            for i in range(4):
                if f1 :
                    x1 += corners[idx1][0][i][0]
                    y1 += corners[idx1][0][i][1]
                    # print("id 1 aruco points : ")
                    # print(corners[idx1][0][i][0])
                    # print(corners[idx1][0][i][1])
                    # print("*******************")
                else : 
                    x1 = 4 * 1000
                    y1 = 4* 1000
            print("Aruco 1 slope : ")
            print(Aruco1Slope)
            theta1 = math.atan(Aruco1Slope)
            degree=theta1*(180/math.pi)
            print("Aruco 1 theta")
            print(degree)
            x1 /= 4
            y1 /= 4
            print("X1: ")
            print(x1)
            print("Y1: ")
            print(y1)
            theta = 0.0
            for i in range(4):
                x2 += corners[idx2][0][i][0]
                y2 += corners[idx2][0][i][1]
            x2 /= 4
            y2 /= 4
            # print("X2: ")
            # print(x2)
            # print("Y2: ")
            # print(y2)
            for i in range(4):
                x3 += corners[idx3][0][i][0]
                y3 += corners[idx3][0][i][1]
            x3 /= 4
            y3 /= 4
            # print("X3: ")
            # print(x3)
            # print("Y3: ")
            # print(y3)
            # for i in range(4) :
            #     print("id 4 aruco marker x")
            #     print(corners[idx4][0][i][0])
            #     print("id 4 aruco marker y")
            #     print(corners[idx4][0][i][1])

            pose_msg = Pose2D()
            pose_msg.x = x1
            pose_msg.y = y1
            pose_msg.theta = theta
            self.publisher_1.publish(pose_msg)
            pose_msg.x = x2
            pose_msg.y = y2
            pose_msg.theta = theta
            self.publisher_2.publish(pose_msg)
            pose_msg.x = x3
            pose_msg.y = y3
            pose_msg.theta = theta
            self.publisher_3.publish(pose_msg)
            time.sleep(0.5)
            theta1Aruco = Float32()
            theta1Aruco.data = theta1
            self.publisherAngleAruco1.publish(theta1Aruco)
                
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")


def main(args=None):

    rclpy.init(args=args)
    
   
    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()