#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Tries to estimates iceberg velocity.
#tony.jacob@uri.edu

#ros2 bag record /alpha_rise/costmap/local/image /alpha_rise/odometry/filtered /alpha_rise/path/state /alpha_rise/stonefish/msis/data/pointcloud/filtered /alpha_rise/fls/pointcloud /iceberg/odometry /tf
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import TransformStamped, Vector3
import tf2_ros
from rclpy.time import Time
import math
from rclpy.parameter import Parameter

class Loop(Node):
    def __init__(self):
        super().__init__('loop_checker')

        # Subscibers
        self.create_subscription(Image, "/alpha_rise/costmap/local/image", self.costmap_image_callback, 10)
        self.create_subscription(Odometry, "/alpha_rise/odometry/filtered", self.odometry_callback, 10)
        self.create_subscription(Int16 ,"/alpha_rise/path/state", self.state_callback, 10)

        # Publishers
        self.map_pub = self.create_publisher(Image, "/alpha_rise/costmap/global/image", 10)
        self.image_match_pub = self.create_publisher(Image, "/alpha_rise/costmap/local/match", 10)
        self.revisit = self.create_publisher(Image, "/alpha_rise/costmap/global/match", 10)
        self.iceberg_odom_pub = self.create_publisher(Odometry, "/alpha_rise/iceberg/odometry", 10)

        # Params
        self.declare_parameter('moment_comparison_threshold', Parameter.Type.DOUBLE)
        self.declare_parameter('revisit_distance_threshold', Parameter.Type.INTEGER)
        self.declare_parameter('revisit_match_threshold', Parameter.Type.INTEGER)
        self.declare_parameter('msis_scan_time', Parameter.Type.DOUBLE)
        self.moment_comparison_threshold = self.get_parameter('moment_comparison_threshold').value
        self.revisit_distance_threshold = self.get_parameter('revisit_distance_threshold').value
        self.revisit_match_threshold = self.get_parameter('revisit_match_threshold').value
        self.msis_scan_time = self.get_parameter('msis_scan_time').value

        self.map = np.zeros((700, 700), dtype=np.uint8)
        
        self.bridge = CvBridge()
        self.akaze = cv2.AKAZE_create()
        # AKAZE uses binary descriptors, so Hamming distance is still valid
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.prev_cx_centered = 0
        self.prev_cy_centered = 0

        self.odom_check = False
        self.state = -1
        self.iceberg_tf = False

        self.list_of_local_maps = []
        self.list_of_vx_in_odom = []
        self.list_of_iceberg_frame_time_in_odom = []

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer,self)
        self.br = tf2_ros.TransformBroadcaster(self)
    
    def state_callback(self, msg):
        self.state = msg.data

    def odometry_callback(self, msg):
        self.vx_x = msg.pose.pose.position.x
        self.vx_y = msg.pose.pose.position.y
        self.vx_z = msg.pose.pose.position.z
        """
        ------->x IMAGE
        |       x
        |    _|
        |    y ODOM
        v y
        """
        self.vx_x_image = -round(self.vx_y)
        self.vx_y_image = -round(self.vx_x)
        self.odom_check = True

    def costmap_image_callback(self, msg):
            #Only consider on following mode
            if (self.odom_check) and (self.state == 0):
                self.iceberg_frame_time_in_odom = msg.header.stamp
                self.resolution = 1.0
                costmap_cv_image = self.bridge.imgmsg_to_cv2(msg)

                ## Global Costmap Image
                self.map, updated = self.create_global_map(self.map, costmap_cv_image, (self.vx_x_image, self.vx_y_image))
                self.map_pub.publish(self.bridge.cv2_to_imgmsg(self.map))
                
                # Match Maps
                if updated:
                    self.calculate_velocity()
                    self.is_revisit(costmap_cv_image, self.list_of_local_maps, len(self.list_of_local_maps)//2)

    def calculate_velocity(self):
        if len(self.list_of_local_maps)  > 1:
            recent = cv2.resize(self.list_of_local_maps[-1],(300,300), interpolation=cv2.INTER_CUBIC)
            latest =cv2.resize(self.list_of_local_maps[-2],(300,300), interpolation=cv2.INTER_CUBIC)

            kp1,des1 = self.akaze.detectAndCompute(recent, None)
            kp2,des2 = self.akaze.detectAndCompute(latest, None)

            # Match descriptors
            matches = self.bf.match(des1, des2)

            # Sort matches by descriptor distance
            matches = sorted(matches, key=lambda x: x.distance)

            # Extract matched keypoints
            pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
            pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
            
            # Estimate affine transform
            matrix, inliers = cv2.estimateAffinePartial2D(pts1, pts2)

            if matrix is None:
                raise RuntimeError("Affine transformation could not be estimated")

            # Extract translation and rotation from affine matrix
            dx = matrix[0, 2]
            dy = matrix[1, 2]

            # Calculate rotation angle in radians
            yaw = np.arctan2(matrix[1, 0], matrix[0, 0])

            # Convert to Odom Frame
            """
            ------->x IMAGE
            |       x
            |    _|
            |    y ODOM
            v y
            """
            #Odometry X = - Image Y * resolution/3 
            #Odometry Y = - Image X * resolution/3
            #Odometry Yaw = pi/2 - Image Yaw
            odom_dx = -dy/3*self.resolution
            odom_dy = -dx/3*self.resolution
            odom_yaw = -(yaw + math.pi / 2)

            delta_time = self.list_of_iceberg_frame_time_in_odom[-1].sec - self.list_of_iceberg_frame_time_in_odom[-2].sec

            x_dot = odom_dx/delta_time
            y_dot = odom_dy/delta_time
            yaw_dot = odom_yaw/delta_time

            iceberg_odom_msg = Odometry()
            iceberg_odom_msg.header.frame_id = "alpha_rise/odom"
            iceberg_odom_msg.header.stamp = self.list_of_iceberg_frame_time_in_odom[-1]

            iceberg_odom_msg.child_frame_id = "alpha_rise/iceberg"
            iceberg_odom_msg.pose.pose.position.x = self.edge_odom_tf.transform.translation.x
            iceberg_odom_msg.pose.pose.position.y = self.edge_odom_tf.transform.translation.y
            iceberg_odom_msg.pose.pose.position.z = self.edge_odom_tf.transform.translation.z
            iceberg_odom_msg.twist.twist.linear = Vector3(x=x_dot, y=y_dot, z=0.0)
            iceberg_odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=yaw_dot)

            self.iceberg_odom_pub.publish(iceberg_odom_msg)

            # Draw good matches
            matched_img = cv2.drawMatches(
                recent, kp1,
                latest, kp2,
                matches, None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            self.image_match_pub.publish(self.bridge.cv2_to_imgmsg(matched_img))
    
    def compare_moment(self, local, best, threshold):
        """
        Compare two images using Hu Moments to determine similarity.

        Args:
            local (ndarray): First image (grayscale or binary).
            best (ndarray): Second image to compare.
            threshold (float): Distance threshold below which images are considered similar.

        Returns:
            is_similar (bool): True if Hu moment distance < threshold.
            hu_distance (float): The computed Hu moment distance.
        """

        def preprocess_and_get_hu(img):
            # Convert to binary if needed (assumes grayscale input)
            _, binary = cv2.threshold(img, 10, 255, cv2.THRESH_BINARY)
            # Compute moments
            moments = cv2.moments(binary)
            # Compute Hu moments
            hu = cv2.HuMoments(moments)
            # Log sca le for comparison (add epsilon to avoid log(0))
            hu_log = -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)
            return hu_log

        # Get Hu moments
        hu_local = preprocess_and_get_hu(local)
        hu_best = preprocess_and_get_hu(best)

        # Compute Euclidean distance between Hu moment vectors
        hu_distance = np.linalg.norm(hu_local - hu_best)
        # Compare to threshold
        is_similar = hu_distance < threshold

        return is_similar, hu_distance
    
    def is_revisit(self, query_img, img_list, oldest_n):
        if oldest_n > 4:
            query_img = cv2.resize(query_img, (300,300),interpolation=cv2.INTER_CUBIC)
            recent_image_kp, des1 = self.akaze.detectAndCompute(query_img,None)

            best_index = -1
            best_matches_count = 0
            best_matches = None
            best_image_kp2 = None

            if oldest_n <= 0:
                return None # invalid parameter

            end_idx = min(oldest_n, len(img_list))  # In case oldest_n > total images

            for i in range(end_idx):
                img = img_list[i]
                kp2, des2 = self.akaze.detectAndCompute(cv2.resize(img, (300,300),interpolation=cv2.INTER_CUBIC), None)
                if des2 is None:
                    continue

                matches = self.bf.match(des1, des2)
                matches = sorted(matches, key=lambda x: x.distance)
                good_matches = [m for m in matches if m.distance <self.revisit_distance_threshold] # lower is better.

                if len(good_matches) > best_matches_count:
                    best_matches_count = len(good_matches)
                    best_index = i
                    best_matches = good_matches
                    best_image_kp2 = kp2

            if best_index == -1 or best_matches_count < self.revisit_match_threshold:  # Minimum 2 matches
                return None
            else:
                is_similar, hu_distance = self.compare_moment(query_img, img_list[best_index], self.moment_comparison_threshold)
                if is_similar:
                    match_img = cv2.drawMatches(query_img, recent_image_kp, cv2.resize(img_list[best_index],(300,300), interpolation=cv2.INTER_CUBIC), best_image_kp2, best_matches, None,
                                                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                    
                    self.revisit.publish(self.bridge.cv2_to_imgmsg(match_img))
                    print(hu_distance, self.state)


    def create_global_map(self, large_img, small_img, small_center_coords):
        large_image_copy = large_img.copy()
        lh, lw = large_img.shape[:2]
        sh, sw = small_img.shape[:2]
        
        # Coordinates where the center of the small image should go (origin at large image center)
        cx_centered, cy_centered = small_center_coords
        
        # Convert to top-left origin pixel coordinates
        cx = lw // 2 + cx_centered
        cy = lh // 2 + cy_centered 

        # Compute top-left corner for placing the small image
        x_start = cx - sw // 2
        y_start = cy - sh // 2
        x_end = x_start + sw
        y_end = y_start + sh

        # Ensure bounds
        if x_start < 0 or y_start < 0 or x_end > lw or y_end > lh:
            raise ValueError("Small image goes out of bounds of large image at given center.")

        mask = small_img > 0
        large_img[y_start:y_end, x_start:x_end][mask] = small_img[mask]
        updated = False
       
        if self.iceberg_tf == False:
            self.start_time = self.iceberg_frame_time_in_odom
            #Publish TF
            self.edge_odom_tf = self.tf_buffer.lookup_transform("alpha_rise/costmap/edge_frame", "alpha_rise/odom", 
                                                              time=Time())
            odom_costmap_tf = TransformStamped()
            odom_costmap_tf.header.stamp = self.start_time
            odom_costmap_tf.header.frame_id = 'alpha_rise/odom'
            odom_costmap_tf.child_frame_id = 'alpha_rise/iceberg'
            odom_costmap_tf.transform.translation.x = self.edge_odom_tf.transform.translation.x
            odom_costmap_tf.transform.translation.y = self.edge_odom_tf.transform.translation.y
            odom_costmap_tf.transform.translation.z = np.float64(0)#self.edge_odom_tf.transform.translation.z
            odom_costmap_tf.transform.rotation.x = np.float64(0)#self.edge_odom_tf.transform.rotation.x
            odom_costmap_tf.transform.rotation.y = np.float64(0)#self.edge_odom_tf.transform.rotation.y
            odom_costmap_tf.transform.rotation.z = np.float64(0)#self.edge_odom_tf.transform.rotation.z
            odom_costmap_tf.transform.rotation.w = np.float64(1)#self.edge_odom_tf.transform.rotation.w
            self.br.sendTransform(odom_costmap_tf)

            large_image_copy = large_img
            
            self.prev_cx_centered = cx_centered
            self.prev_cy_centered = cy_centered

            self.list_of_local_maps.append(small_img)
            self.list_of_vx_in_odom.append((self.vx_x, self.vx_y))
            self.list_of_iceberg_frame_time_in_odom.append(self.iceberg_frame_time_in_odom)
            updated = True

            print(f"updated; {len(self.list_of_local_maps), len(self.list_of_iceberg_frame_time_in_odom), len(self.list_of_vx_in_odom)}", flush=True)
            self.iceberg_tf = True

        if self.iceberg_frame_time_in_odom.sec - self.start_time.sec > 26: #s
            self.start_time = self.iceberg_frame_time_in_odom
            large_image_copy = large_img
            
            self.prev_cx_centered = cx_centered
            self.prev_cy_centered = cy_centered

            self.list_of_local_maps.append(small_img)
            self.list_of_vx_in_odom.append((self.vx_x, self.vx_y))
            self.list_of_iceberg_frame_time_in_odom.append(self.iceberg_frame_time_in_odom)
            updated = True

            print(f"updated; {len(self.list_of_local_maps), len(self.list_of_iceberg_frame_time_in_odom), len(self.list_of_vx_in_odom)}", flush=True)
        
        return large_image_copy, updated

    
def main():
    rclpy.init()
    node = Loop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()