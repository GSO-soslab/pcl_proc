#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Tries to estimates iceberg velocity.
#tony.jacob@uri.edu

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge
import cv2
import math
class Loop(Node):
    def __init__(self):
        super().__init__('loop_checker')

        # Subscibers
        self.create_subscription(Image, "/alpha_rise/costmap/local/image", self.costmap_image_callback, 10)
        self.create_subscription(Odometry, "/alpha_rise/odometry/filtered", self.odometry_callback, 10)

        # Publishers
        self.map_pub = self.create_publisher(Image, "/alpha_rise/costmap/global/image", 10)
        self.map_pub_list = self.create_publisher(Image, "/alpha_rise/costmap/local/match", 10)


        self.map = np.zeros((700, 700), dtype=np.uint8)
        
        self.bridge = CvBridge()
        self.akaze = cv2.AKAZE_create()
        # AKAZE uses binary descriptors, so Hamming distance is still valid
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.prev_cx_centered = 0
        self.prev_cy_centered = 0

        self.list_of_local_maps = []
        self.list_of_vx_in_odom = []
        self.list_of_iceberg_frame_time_in_odom = []

    def odometry_callback(self, msg):
        self.vx_x = msg.pose.pose.position.x
        self.vx_y = msg.pose.pose.position.y

        """
        ------->x IMAGE
        |       x
        |    _|
        |    y ODOM
        v y
        """
        self.vx_x_image = -round(self.vx_y)
        self.vx_y_image = -round(self.vx_x)
        
    def costmap_image_callback(self, msg):
            self.iceberg_frame_time_in_odom = msg.header.stamp
            costmap_cv_image = self.bridge.imgmsg_to_cv2(msg)

            ## Global Costmap Image
            self.map = self.create_global_map(self.map, costmap_cv_image, (self.vx_x_image, self.vx_y_image))
            self.map_pub.publish(self.bridge.cv2_to_imgmsg(self.map))
            
            # Match Maps
            self.match_maps(costmap_cv_image)

    def match_maps(self, local_image):
        if len(self.list_of_local_maps)  > 4:
            index, best_image,  kp1, best_kp2, best_matches = self.most_similar_image_akaze(local_image, self.list_of_local_maps, len(self.list_of_local_maps)//2)
            
            if index != None:
                mix = self.bridge.cv2_to_imgmsg(best_image, encoding="rgb8")
                self.map_pub_list.publish(mix)
            
                x_dot, y_dot, psi_dot = self.calculate_velocity(index, kp1, best_kp2, best_matches)
                
    def calculate_velocity(self, index, kp1, kp2, matches):
        ##FFEAT: Get time from costmap image msg.
        ## Affine transform gets you delta x and delta y in pixel.
        ## Get velocities that way.
        delta_time = self.iceberg_frame_time_in_odom.sec - self.list_of_iceberg_frame_time_in_odom[index][0].sec

        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        M, inliers = cv2.estimateAffinePartial2D(pts1, pts2)
        """
        [
        cos(theta)*s, -sin(theta)*s, tx;
        sin(theta)*s,  cos(theta)*s, ty
        ]
        """
        
        #rad = atan2(sin, cos)
        angle_rad = math.atan2(M[0,1], M[0,0]) #Image Frame

    def most_similar_image_akaze(self, query_img, img_list, oldest_n):
        kp1, des1 = self.akaze.detectAndCompute(query_img, None)

        best_index = -1
        best_matches_count = 0
        best_matches = None
        best_kp2 = None

        if oldest_n <= 0:
            return None, None  # invalid parameter

        end_idx = min(oldest_n, len(img_list))  # In case oldest_n > total images

        for i in range(end_idx):
            img = img_list[i]
            kp2, des2 = self.akaze.detectAndCompute(img, None)
            if des2 is None:
                continue

            matches = self.bf.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)
            good_matches = [m for m in matches if m.distance < 70] # lower is better.

            if len(good_matches) > best_matches_count:
                best_matches_count = len(good_matches)
                best_index = i
                best_matches = good_matches
                best_kp2 = kp2

        if best_index == -1 or best_matches_count < 2:  # Minimum 2 matches
            return None, None

        # angle = self.calculate_rotation_from_matches(kp1, best_kp2, best_matches)

        match_img = cv2.drawMatches(query_img, kp1, img_list[best_index], best_kp2, best_matches, None,
                                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        
        return best_index, match_img, kp1, best_kp2, best_matches

    def create_global_map(self, large_img, small_img, small_center_coords):
        large_image_copy = large_img.copy()
        lh, lw = large_img.shape[:2]
        sh, sw = small_img.shape[:2]
        
        # Coordinates where the center of the small image should go (origin at large image center)
        cx_centered, cy_centered = small_center_coords
        curr_distance = np.sqrt((cx_centered - self.prev_cx_centered)**2 + (cy_centered - self.prev_cy_centered)**2)
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

        if curr_distance > 20: #m
            large_image_copy = large_img
            
            self.prev_cx_centered = cx_centered
            self.prev_cy_centered = cy_centered

            self.list_of_local_maps.append(small_img)
            self.list_of_vx_in_odom.append((self.vx_x, self.vx_y))
            self.list_of_iceberg_frame_time_in_odom.append(self.iceberg_frame_time_in_odom)

            print(f"updated; {len(self.list_of_local_maps), len(self.list_of_iceberg_frame_time_in_odom), len(self.list_of_vx_in_odom)}", flush=True)
        
        return large_image_copy

    
def main():
    rclpy.init()
    node = Loop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()