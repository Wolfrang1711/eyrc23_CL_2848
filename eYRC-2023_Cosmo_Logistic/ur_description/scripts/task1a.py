#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ CL#2848 ]
# Author List:		[ Pratik ]
# Filename:		    task1a.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CompressedImage, Image

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    area = None
    width = None
    
    (topLeft, topRight, bottomRight, bottomLeft) = coordinates
    
    # convert each of the (x, y)-coordinate pairs to integers
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))                 
    
    # Calculate the width and height of the marker
    width = np.sqrt((topRight[0] - topLeft[0])**2 + (topRight[1] - topLeft[1])**2)
    height = np.sqrt((topRight[0] - bottomRight[0])**2 + (topRight[1] - bottomRight[1])**2)
    
    # Calculate the area of the marker
    area = width * height

    return area, width

def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    # variable as a threshold value to detect aruco markers of certain size.
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    camera_matrix = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    distance_matrix = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    marker_size = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    ids = []
    aruco_info_list = []
    rotation_matrix = np.zeros((3, 3))
    
    axis_length = 0.1

    # gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_image = image
    
    # defining aruco dictionary and parameters
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters()

    # finding corners and aruco ids
    corners, ids, _ = cv2.aruco.detectMarkers(gray_image, arucoDict, parameters=arucoParams)	

    if len(corners) > 0:
        
        # flatten the ArUco IDs list
        ids = ids.flatten()
        
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
                        
            # extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            # convert each of the (x,y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
                        
            # draw the bounding box of the ArUCo detection
            cv2.line(gray_image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(gray_image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(gray_image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(gray_image, bottomLeft, topLeft, (0, 255, 0), 2)
            
            # draw the ArUco marker ID on the frame
            cv2.putText(gray_image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
            area, width = calculate_rectangle_area(corners)
                            
            if(area > aruco_area_threshold):
   
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(gray_image, (cX, cY), 4, (0, 0, 255), -1)
                                
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix, distance_matrix)
                                
                # Get the rotation matrix from the rotation vector
                rotation_matrix, _ = cv2.Rodrigues(rvecs)
                
                # Create a Rotation object from the rotation matrix
                r = Rotation.from_matrix(rotation_matrix)

                # Convert the rotation to Euler angles (in radians)  'zyx' specifies the order of rotations
                yaw, _, _ = r.as_euler('zyx', degrees=False) 
                
                cv2.drawFrameAxes(image, camera_matrix, distance_matrix, rotation_matrix, tvecs[0], axis_length)
                
                # Store ArUco marker information in the list
                aruco_info_list.append({'ID': markerID,
                                        'Center': (cX, cY),
                                        'Angle' : yaw,
                                        'Distance': tvecs[0][0][2],
                                        'Width': width})
    
    return aruco_info_list

class aruco_tf(Node):
    '''
    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
        '''
        
        super().__init__('aruco_tf_publisher')                                          # registering node

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.rgb_image = None                                                           # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())           
            
    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''
        
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''
        
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''      
        
        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        
        aruco_info_list = []
            
        # Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center'
        aruco_info_list = detect_aruco(self.rgb_image)
        
        cv2.imshow("rgb_image", self.rgb_image)
        cv2.waitKey(1)

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation
    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node
    node.get_logger().info('Node created: Aruco tf process')        # logging information
    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'
    
    try:
        rclpy.spin(aruco_tf_class)                                  # spining on the object to make it alive in ROS 2 DDS
        
    except KeyboardInterrupt:
        pass
    
    finally:    
        aruco_tf_class.destroy_node()                               # destroy node after spin ends
        rclpy.try_shutdown()                                        # shutdown process
        
if __name__ == '__main__':
     
    main()