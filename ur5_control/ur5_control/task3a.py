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

def calculate_rectangle_area(coordinates):
    (x1, y1) = coordinates[0][0], coordinates[0][1]  # Top-left
    (x2, y2) = coordinates[1][0], coordinates[1][1]  # Top-right
    (x3, y3) = coordinates[2][0], coordinates[2][1]  # Bottom-right
    (x4, y4) = coordinates[3][0], coordinates[3][1]  # Bottom-left

    width = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    height = math.sqrt((x4 - x1)**2 + (y4 - y1)**2)

    area = width * height
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
    aruco_area_threshold = 1500
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    camera_matrix = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])


    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids=[]
    tvecs= []
    corner_list = []

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    k1 = 0.1
    k2 = 0.1
    p1 = 0.1
    p2 = 0.1
    k3 = 0.1
    dist_coeffs = np.array([k1, k2, p1, p2, k3])

    # Image dimensions (width, height)
    img_size = (1280, 720)

    # Compute undistortion map
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, img_size, alpha=1)
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, img_size, cv2.CV_32FC1)

    # Convert to grayscale (if necessary)

    #gray = cv2.remap(gray, mapx, mapy, cv2.INTER_LINEAR)
    #depth_image = cv2.remap(depth_image, mapx, mapy, cv2.INTER_NEAREST)  # Use nearest-neighbor for depth



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

        corner_list.append(coordinates)


        width_aruco_list.append(width)

        # Calculate the center of the marker
        center_x = np.mean(coordinates[:, 0])
        center_y = np.mean(coordinates[:, 1])
        center_aruco_list.append((center_x, center_y))

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids,tvecs, corner_list

class aruco_tf(Node):
    def __init__(self):
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
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def colorimagecb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if(self.cv_image is not None and self.depth_image is not None):
            self.process_image()

    def get_final_quaternion(self,yaw_angle_deg):
        pitch_angle = np.deg2rad(90)  # Fixed pitch angle in radians
        yaw_angle = np.deg2rad(yaw_angle_deg)  # Convert input yaw angle to radians
        ang=np.deg2rad(0)
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
            rotation_vector_x = np.array([1, 0, 0]) * yaw_angle  # Rotation about z-axis
            rotation_matrix_x, _ = cv2.Rodrigues(rotation_vector_x)  # Convert vector to rotation matrix
            return rotation_matrix_x
        # Get rotation matrices
        R_pitch = rotation_matrix_y(pitch_angle)
        R_z = rotation_matrix_z(yaw_angle)
        R_x = rotation_matrix_x(ang)
        # Composite rotation: Apply pitch first, then yaw
        R_total = R_pitch @ R_z  # Matrix multiplication
        R_total = R_total @ R_x
        # Convert the resulting rotation matrix to a quaternion
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
        # Get quaternion from base to camera using roll, pitch, yaw
        q_bc = self.quaternion_from_euler(camera_rpy[0], camera_rpy[1], camera_rpy[2])

        # Get quaternion from base to object
        q_bo = self.quaternion_multiply(q_bc, camera_to_object_quat)

        return q_bo.as_quat()  # Returns [x, y, z, w]

    def point_in_main_frame(self,point_tilted):
        # Fixed position of the tilted frame in the main frame
        frame_position = np.array([1.152, 0.020, 1.238])

        # Fixed roll, pitch, and yaw angles in degrees
        roll = 0.0
        pitch = 47.80
        yaw = 179.909

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
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids,tvecss, corner_list = detect_aruco(self.cv_image)
        cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

        dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

        print("IDS:", ids)

        if len(ids) > 0:
            for i in range(len(ids)):
                try:
                    mainX, mainY = corner_list[i][0]
                    axisX_X, axisX_Y = corner_list[i][1]
                    axisY_X, axisY_Y = corner_list[i][3]

                    depth0 = self.depth_image[int(mainY), int(mainX)] / 1000
                    depth1 = self.depth_image[int(axisX_Y), int(axisX_X)] / 1000
                    depth3 = self.depth_image[int(axisY_Y), int(axisY_X)] / 1000

                    x0 = - depth0 * (mainX - centerCamX) / focalX
                    y0 = - depth0 * (mainY - centerCamY) / focalY
                    z0 = depth0

                    x1 = - depth1 * (axisX_X - centerCamX) / focalX
                    y1 = - depth1 * (axisX_Y - centerCamY) /focalY
                    z1 = depth1

                    x3 = - depth3 * (axisY_X - centerCamX) / focalX
                    y3 = - depth3 * (axisY_Y - centerCamY) / focalY
                    z3 = depth3

                    x0, y0, z0 = self.point_in_main_frame([z0, x0, y0])
                    x1, y1, z1 = self.point_in_main_frame([z1, x1, y1])
                    x3, y3, z3 = self.point_in_main_frame([z3, x3, y3])

                    red_x = x0 - x1
                    red_y = y0 - y1
                    red_z = z0 - z1

                    green_x = x0 - x3
                    green_y = y0 - y3
                    green_z = z0 - z3

                    red = np.array([red_x, red_y, red_z])
                    green = np.array([green_x, green_y, green_z])

                    # Compute the cross product to find the blue vector
                    blue = np.cross(red, green)

                    # Normalize the vectors
                    red_norm = red / np.linalg.norm(red)
                    green_norm = green / np.linalg.norm(green)
                    blue_norm = blue / np.linalg.norm(blue)

                    # Form the rotation matrix
                    rotation_matrix = np.column_stack((red_norm, green_norm, blue_norm))

                    # Convert the rotation matrix to a quaternion
                    rot = R.from_matrix(rotation_matrix)
                    box_quat = rot.as_quat()

                    cX = center_aruco_list[i][0]
                    cY= center_aruco_list[i][1]

                    distance_from_rgb = self.depth_image[int(cY),int(cX)] / 1000

                    x = - distance_from_rgb * (cX - centerCamX) / focalX
                    y = - distance_from_rgb * (cY - centerCamY) / focalY
                    z = distance_from_rgb

                    a,b,c=self.point_in_main_frame([z,x,y])

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'base_link'

                    t.child_frame_id = f'obj_{ids[i][0]}'
                    t.transform.translation.x = a
                    t.transform.translation.y = b
                    t.transform.translation.z = c


                    t.transform.rotation.x = box_quat[0]
                    t.transform.rotation.y = box_quat[1]
                    t.transform.rotation.z = box_quat[2]
                    t.transform.rotation.w = box_quat[3]

                    self.br.sendTransform(t)

                except IndexError as e:
                    print(e)

            cv2.imshow("Aruco Detection", self.cv_image)
            cv2.waitKey(1)

def main():
    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')
         # logging information
    print("NEW BUILD CHECKEr 23232")

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
