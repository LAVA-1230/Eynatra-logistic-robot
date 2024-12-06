#!/usr/bin/python3

# -*- coding: utf-8 -*-



'''

*****************************************************************************************

*

*        		===============================================

*           		    Logistic coBot (LB) Theme (eYRC 2024-25)

*        		===============================================

*

*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).

*

*  This software is made available on an "AS IS WHERE IS BASIS".

*  Licensee/end user indemnifies and will keep e-Yantra indemnified from

*  any and all claim(s) that emanate from the use of the Software or

*  breach of the terms of this agreement.

*

*****************************************************************************************

'''



# Team ID:          [ Team-ID ]

# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]

# Filename:		    task1b_boiler_plate.py

# Functions:

#			        [ Comma separated list of functions in this file ]

# Nodes:		    Add your publishing and subscribing node

#			        Publishing Topics  - [ /tf ]

#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]





################### IMPORT MODULES #######################



import rclpy

import sys

import cv2

import math

import tf2_ros

import numpy as np

from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import CompressedImage, Image

from geometry_msgs.msg import Quaternion





##################### FUNCTION DEFINITIONS #######################



def calculate_rectangle_area(coordinates):

    '''

    Description:    Function to calculate area or detected aruco



    Args:

        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)



    Returns:

        area        (float):    area of detected aruco

        width       (float):    width of detected aruco

    '''



    ############ Function VARIABLES ############



    # You can remove these variables after reading the instructions. These are just for sample.



    # area = None

    # width = None







    ############ ADD YOUR CODE HERE ############



    (x1, y1) = coordinates[0][0], coordinates[0][1]  # Top-left

    (x2, y2) = coordinates[1][0], coordinates[1][1]  # Top-right

    (x3, y3) = coordinates[2][0], coordinates[2][1]  # Bottom-right

    (x4, y4) = coordinates[3][0], coordinates[3][1]  # Bottom-left



    width = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)



    height = math.sqrt((x4 - x1)**2 + (y4 - y1)**2)



    area = width * height



    # INSTRUCTIONS & HELP : 

    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 

    #       and use these coordinates to calculate area and width of aruco detected.

    #	->  Extract values from input set of 4 (x,y) coordinates 

    #       and formulate width and height of aruco detected to return 'area' and 'width'.



    ############################################



    return area, width

def rotate_pose_90_deg_ccw(rotation_vec, translation_vec):

    # Rotation matrix for 90 degrees counterclockwise about Z-axis

    rotation_90_z = np.array([[0, -1, 0],

                              [1,  0, 0],

                              [0,  0, 1]])



    # Convert rotation vector to a rotation matrix

    rotation_mat, _ = cv2.Rodrigues(rotation_vec)

    

    # Apply the rotation matrix

    rotated_rotation_mat = rotation_90_z @ rotation_mat

    rotated_translation_vec = rotation_90_z @ translation_vec

    

    # Convert back to a rotation vector

    rotated_rotation_vec, _ = cv2.Rodrigues(rotated_rotation_mat)

    

    return rotated_rotation_vec, rotated_translation_vec





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



    ############ Function VARIABLES ############
    # ->  You can remove these variables if needed. These are just for suggestions to let you get started
    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500
    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])
    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15
    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids=[]
    tvecs= []
    rvecs=[]
    ############ ADD YOUR CODE HERE ############
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    det=cv2.aruco.ArucoDetector(aruco_dict,aruco_params)
    corners, ids, rejected = det.detectMarkers(gray_image)
    if ids is None:
        return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    for i, marker_corners in enumerate(corners):
        coordinates = marker_corners[0]
        area, width = calculate_rectangle_area(coordinates)
        # Skip markers that are too small (beyond the reach threshold)
        if area < aruco_area_threshold:
            continue
        width_aruco_list.append(width)
        # Calculate the center of the marker
        center_x = np.mean(coordinates[:, 0])
        center_y = np.mean(coordinates[:, 1])
        center_aruco_list.append((center_x, center_y))
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corners, size_of_aruco_m, cam_mat, dist_mat)
        # rvec , tvec = rotate_pose_90_deg_ccw(rvec[0],tvec[0])
        tvecs.append(tvec)
        rvecs.append(rvec)
        # print(rvec)
        # Calculate the distance of the marker from the camera
        distance = np.linalg.norm(tvec)
        distance_from_rgb_list.append(distance)
        # Calculate the angle of the marker in degrees from the rotation vector
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        r = R.from_matrix(rotation_matrix)
        angles = r.as_euler('xyz', degrees=True)
        angle_aruco_list.append(angles)

    # INSTRUCTIONS & HELP : 



    #	->  Convert input BGR image to GRAYSCALE for aruco detection



    #   ->  Use these aruco parameters-

    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)



    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'

    #       ->  HINT: Handle cases for empty markers detection. 



    #   ->  Draw detected marker on the image frame which will be shown later



    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))



    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined



    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker

    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids,tvecs,rvecs


##################### CLASS DEFINITION #######################



class aruco_tf(Node):

    '''

    ___CLASS___



    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.

    '''



    def __init__(self):

        '''

        Description:    Initialization of class aruco_tf

                        All classes have a function called __init__(), which is always executed when the class is being initiated.

                        The __init__() function is called automatically every time the class is being used to create a new object.

                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp

        '''



        super().__init__('aruco_tf_publisher')                                          # registering node



        ############ Topic SUBSCRIPTIONS ############



        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)

        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)



        ############ Constructor VARIABLES/OBJECTS ############



        image_processing_rate = 1                                                   # rate of time to process image (seconds)

        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion

        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms

        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id

        # self.timer = self.create_timer(image_processing_rate,self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())

        self.depth_image = None                                                         # depth image variable (from depthimagecb())





    def depthimagecb(self, data):

        '''

        Description:    Callback function for aligned depth camera topic. 

                        Use this function to receive image depth data and convert to CV2 image



        Args:

            data (Image):    Input depth image frame received from aligned depth camera topic



        Returns:

        '''



        ############ ADD YOUR CODE HERE ############



        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")



        # INSTRUCTIONS & HELP : 



        #	->  Use data variable to convert ROS Image message to CV2 Image type



        #   ->  HINT: You may use CvBridge to do the same



        ############################################





    def colorimagecb(self, data):

        '''

        Description:    Callback function for colour camera raw topic.

                        Use this function to receive raw image data and convert to CV2 image



        Args:

            data (Image):    Input coloured raw image frame received from image_raw camera topic



        Returns:

        '''



        ############ ADD YOUR CODE HERE ############



        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        if(self.cv_image is not None and self.depth_image is not None):

            self.process_image()

                ############ NEECHE WALI LINE DEKHO ############



        # mene flipping and rotate wala code nhi likha h (isme check kaise krenge ki image flipped or rotated h)







        # INSTRUCTIONS & HELP : 



        #	->  Use data variable to convert ROS Image message to CV2 Image type



        #   ->  HINT:   You may use CvBridge to do the same

        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.

        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same



        ############################################



    def get_final_quaternion(self,yaw_angle_deg):
        """
        Takes the yaw angle (in degrees) as input and returns the final quaternion 
        after rotating about the pitch by -14.95 degrees and the yaw by the given angle.
        Args:
        yaw_angle_deg (float): Yaw angle in degrees for rotation around the z-axis.
        Returns:
        numpy.ndarray: Final quaternion in the format [x, y, z, w].
        """
        # Convert degrees to radians
        yaw_angle2=np.deg2rad(179.90)
        pitch_angle = np.deg2rad(47.80)  # Fixed pitch angle in radians
        yaw_angle1 = np.deg2rad(yaw_angle_deg)  # Convert input yaw angle to radians
        ang=np.deg2rad(90)
        # Function to create rotation matrix around Y-axis (Pitch)
        def rotation_matrix_y(pitch_angle):
            rotation_vector_y = np.array([0, 1, 0]) * pitch_angle  # Rotation about y-axis
            rotation_matrix_y, _ = cv2.Rodrigues(rotation_vector_y)  # Convert vector to rotation matrix
            return rotation_matrix_y
        # Function to create rotation matrix around Z-axis (Yaw)
        def rotation_matrix_z(yaw_angle):
            rotation_vector_z = np.array([0, 0, 1]) * yaw_angle  # Rotation about z-axis
            rotation_matrix_z, _ = cv2.Rodrigues(rotation_vector_z)  # Convert vector to rotation matrix
            return rotation_matrix_z
        def rotation_matrix_x(yaw_angle):
            rotation_vector_x = np.array([1, 0, 0]) * yaw_angle  # Rotation about x-axis
            rotation_matrix_x, _ = cv2.Rodrigues(rotation_vector_x)  # Convert vector to rotation matrix
            return rotation_matrix_x
        # # Get rotation matrices
        # R_yaw1=rotation_matrix_z(yaw_angle1)
        # R_pitch = rotation_matrix_y(pitch_angle)
        # R_yaw2 = rotation_matrix_z(yaw_angle2)
        # R_x = rotation_matrix_x(ang)
        # # print(f"rotation x is {R_x}")
        # # Composite rotation: Apply pitch first, then yaw
        # R_total = R_yaw1 @ R_pitch @ R_yaw2 @ R_x
        # # R_total = R_pitch @ R_z  # Matrix multiplication
        # # R_total = R_total @ R_x
        # # Convert the resulting rotation matrix to a quaternion

        # Get rotation matrices
        R_pitch = rotation_matrix_y(pitch_angle)  # Pitch rotation
        R_yaw1 = rotation_matrix_z(yaw_angle1)  # First yaw rotation
        R_x = rotation_matrix_x(ang)  # X-axis rotation

        # Composite rotation: Apply pitch, then first yaw, then x-axis rotation
        R_intermediate = R_pitch @ R_yaw1 @ R_x

        # Apply the second yaw rotation (179.9 degrees about z-axis)
        R_yaw2 = rotation_matrix_z(yaw_angle2)
        R_total = R_intermediate @ R_yaw2

        rot = R.from_matrix(R_total)
        quat = rot.as_quat()  # Returns [x, y, z, w] format
        return quat
    def quaternion_from_euler(self,roll, pitch, yaw):
        """Convert roll, pitch, yaw (in degrees) to quaternion."""
        return R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    def quaternion_multiply(self,q1, q2):
        """Multiply two quaternions q1 and q2."""
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        return r1 * r2
    def get_base_to_object_quaternion(self,camera_rpy, camera_to_object_quat):
        """
        Calculate the quaternion from the base to the object.
        Args:
        camera_rpy (tuple): Roll, pitch, yaw angles of the camera relative to the base (in degrees).
        camera_to_object_quat (np.ndarray): Quaternion from camera to object in the form [x, y, z, w].
        Returns:
        np.ndarray: Quaternion from base to object in the form [x, y, z, w].
        """
        # Get quaternion from base to camera using roll, pitch, yaw
        q_bc = self.quaternion_from_euler(camera_rpy[0], camera_rpy[1], camera_rpy[2])
        # Get quaternion from base to object
        q_bo = self.quaternion_multiply(q_bc, camera_to_object_quat)
        return q_bo.as_quat()  # Returns [x, y, z, w]
    def point_in_main_frame(self,point_tilted):
        """
        Convert point coordinates from a tilted frame to the main frame.
        Args:
        point_tilted (np.ndarray): The coordinates of the point in the tilted frame [x, y, z].
        Returns:
        np.ndarray: The coordinates of the point in the main frame [x, y, z].
        """
        # Fixed position of the tilted frame in the main frame
        frame_position = np.array([1.152, 0.02, 1.238])
        # Fixed roll, pitch, and yaw angles in degrees
        roll = 0.0
        pitch = 47.8
        yaw = 179.90
        # Convert degrees to radians for rotation calculations
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)
        # Calculate the rotation matrix from roll, pitch, yaw
        R = np.array([
            [np.cos(yaw_rad) * np.cos(pitch_rad),
            np.cos(yaw_rad) * np.sin(pitch_rad) * np.sin(roll_rad) - np.sin(yaw_rad) * np.cos(roll_rad),
            np.cos(yaw_rad) * np.sin(pitch_rad) * np.cos(roll_rad) + np.sin(yaw_rad) * np.sin(roll_rad)],
            [np.sin(yaw_rad) * np.cos(pitch_rad),
            np.sin(yaw_rad) * np.sin(pitch_rad) * np.sin(roll_rad) + np.cos(yaw_rad) * np.cos(roll_rad),
            np.sin(yaw_rad) * np.sin(pitch_rad) * np.cos(roll_rad) - np.cos(yaw_rad) * np.sin(roll_rad)],
            [-np.sin(pitch_rad),
            np.cos(pitch_rad) * np.sin(roll_rad),
            np.cos(pitch_rad) * np.cos(roll_rad)
            ]
        ])
        # Transform the point from the tilted frame to the main frame
        point_rotated = R @ point_tilted  # Rotate the point
        point_main = point_rotated + frame_position  # Translate to the main frame
        return point_main
    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.
        Args:
        Returns:
        '''
        ############ Function VARIABLES ############
        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
    #     ############ ADD YOUR CODE HERE ############
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids,tvecss,rvecs_of_the_box = detect_aruco(self.cv_image)
        if(len(rvecs_of_the_box)>0):
            for i in range(len(rvecs_of_the_box)):
                print(f"printing rvecs ka {i}th element {rvecs_of_the_box[i]}")
            print(len(rvecs_of_the_box))
            
        cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
        dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])
        if len(ids) > 0:
            print(len(ids))
            for i in range(len(ids)):
                cX = center_aruco_list[i][0]
                cY= center_aruco_list[i][1]
                # print(center_aruco_list)  
                distance_from_rgb = self.depth_image[int(cY),int(cX)] / 1000.0
                # distance_from_rgb=distance_from_rgb/np.cos(0.261)
                angle_aruco = angle_aruco_list[i]
                corrected_angle = (0.788 * angle_aruco[2]) - ((angle_aruco[2]**2) / 3160)

                # angle_aruco[2]=(angle_aruco[2]*math.pi)/180
                cv2.circle(self.cv_image,(int(cX),int(cY)),2,(0,0,255),2)
                # r = R.from_euler('xyz', [0, 2*math.pi-0.261, angle_aruco[2]], degrees=True)
                # quat = r.as_quat()
                nq=self.quaternion_from_euler(90+90+45,-180-45,90+45+corrected_angle)
                nqb=self.get_final_quaternion(90+corrected_angle)
                # distance_from_rgb=distance_from_rgb*np.cos(0.261)
                x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
                y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
                z = distance_from_rgb
                # nz=-y+1.04
                # nx=z*np.cos(0.261)-0.97
                # ny=x
                # print(x,y,z)
                # print(tvecss)
                a,b,c=self.point_in_main_frame([z,x,y])
                # bq=self.get_base_to_object_quaternion(np.array([0.0, 14.95, 0.0]),nq)
                # print(z*np.cos(0.261),tvecss[i][0][0][2])
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'    
                t.child_frame_id = f'obj_{ids[i][0]}'
                t.transform.translation.x = a
                t.transform.translation.y = b
                t.transform.translation.z = c
                # translation_vector = np.array([a,b,c]).reshape(1,3)  # Replace with your coordinates
                # print(translation_vector)
                if(ids[i]==49):
                    qx, qy, qz, qw = nq
                    # Convert quaternion to rotation matrix
                    rotation_matrix = np.array([
                        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
                    ])
                    # Convert rotation matrix to rvec using cv2.Rodrigues
                    rvec, _ = cv2.Rodrigues(rotation_matrix)
                    # print(rvec[0])
                    rl=[rvec[0][0],rvec[1][0],rvec[2][0]]
                    rl=np.array(rl).reshape((1,3))
                    # print(rl)
                    t.transform.rotation.x = nq[0]
                    t.transform.rotation.y = nq[1]
                    t.transform.rotation.z = nq[2]
                    t.transform.rotation.w = nq[3]
                    # cv2.drawFrameAxes(self.cv_image, cam_mat, dist_mat, [a,b,c], rl, 0.1)
                else:
                    qx, qy, qz, qw = nq
                    # Convert quaternion to rotation matrix
                    rotation_matrix = np.array([
                        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
                    ])
                    # Convert rotation matrix to rvec using cv2.Rodrigues
                    rvec, _ = cv2.Rodrigues(rotation_matrix)
                    rl=[rvec[0][0],rvec[1][0],rvec[2][0]]
                    rl=np.array(rl).reshape((1,3))
                    rotation =R.from_euler('xyz',[rvec[0][0],rvec[1][0],rvec[2][0]])
                    quaternion = rotation.as_quat()


                    # calculating offset
                    offset=R.from_euler('xyz',[rvecs_of_the_box[i][0][0][0],rvecs_of_the_box[i][0][0][1],rvecs_of_the_box[i][0][0][2]])
                    print(f"printing rvecs ke oth element ke comps : {rvecs_of_the_box[i][0][0][0]} , {rvecs_of_the_box[i][0][0][1]} , {rvecs_of_the_box[i][0][0][2]}")
                    offset_quat=offset.as_quat()
                    # print(rl)
                    print(f"printing rvec{rvec}")
                    print(rvec[0][0])
                    print(f"printing rl{rl}")
                    t.transform.rotation.x = qx+offset_quat[0]
                    t.transform.rotation.y = qy+offset_quat[1]
                    t.transform.rotation.z = qz+offset_quat[2]
                    t.transform.rotation.w = qw+offset_quat[3]
                    # cv2.drawFrameAxes(self.cv_image, cam_mat, dist_mat, [a,b,c], rl, 0.1)
                self.get_logger().info(f"box id:{ids[i][0]} and box position is {a},{b},{c}")
                self.get_logger().info(f"box id:{ids[i][0]} and box orientation is {quaternion[0]},{quaternion[1]},{quaternion[2]},{quaternion[3]}")
                self.br.sendTransform(t)

                # try:

                #     transform = self.tf_buffer.lookup_transform('base_link', f'cam_{ids[i]}', rclpy.time.Time())

                #     base_x = transform.transform.translation.x # x-coordinate of the marker's position relative to the base frame.

                #     base_y = transform.transform.translation.y # y-coordinate of the marker's position relative to the base frame.

                #     base_z = transform.transform.translation.z # z-coordinate of the marker's position relative to the base frame.



                #     t_base = TransformStamped()

                #     t_base.header.stamp = self.get_clock().now().to_msg()

                #     t_base.header.frame_id = 'base_link'

                #     t_base.child_frame_id = f'obj_{ids[i]}'

                #     t_base.transform.translation.x = base_x

                #     t_base.transform.translation.y = base_y

                #     t_base.transform.translation.z = base_z

                #     t_base.transform.rotation.x = transform.transform.rotation.x

                #     t_base.transform.rotation.y = transform.transform.rotation.y

                #     t_base.transform.rotation.z = transform.transform.rotation.z

                #     t_base.transform.rotation.w = transform.transform.rotation.w



                #     self.br.sendTransform(t_base)



                # except Exception as e:

                #     self.get_logger().warn(f"Failed to lookup transform: {str(e)}")



            cv2.imshow("Aruco Detection", self.cv_image)

            cv2.waitKey(1)



        # INSTRUCTIONS & HELP : 



        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above



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



        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1C)

        #               Also, auto eval script will be judging angular difference as well. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)



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



    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS



    aruco_tf_class.destroy_node()                                   # destroy node after spin ends



    rclpy.shutdown()                                                # shutdown process





if __name__ == '__main__':

    '''

    Description:    If the python interpreter is running that module (the source file) as the main program, 

                    it sets the special __name__ variable to have a value “__main__”. 

                    If this file is being imported from another module, __name__ will be set to the module’s name.

                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/

    '''



    main()
