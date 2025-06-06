#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Generates curve, standoff curve of the iceberg from a costmap
#tony.jacob@uri.edu

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
import tf2_ros
import tf_transformations as tf_transform
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from std_msgs.msg import Float32
import numpy as np
import cv2
import math
import scipy.optimize
import os
import sys
import path_utils
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class PathGen(Node):
    def __init__(self):
        super().__init__('path_generator')
        # Declare parameters 
        self.declare_parameter('odom_frame', Parameter.Type.STRING)
        self.declare_parameter('base_frame', Parameter.Type.STRING)
        self.declare_parameter('debug', Parameter.Type.BOOL)
        self.declare_parameter('enable_search_mode', Parameter.Type.BOOL)
        self.declare_parameter('costmap_topic', Parameter.Type.STRING)
        self.declare_parameter('path_topic', Parameter.Type.STRING)
        self.declare_parameter('canny_min_threshold', Parameter.Type.INTEGER)
        self.declare_parameter('canny_max_threshold', Parameter.Type.INTEGER)
        self.declare_parameter('msis_vertical_beamwidth', Parameter.Type.INTEGER)
        self.declare_parameter('standoff_distance_meters', Parameter.Type.DOUBLE)
        self.declare_parameter('points_to_sample_from_curve', Parameter.Type.INTEGER)
        self.declare_parameter('min_scan_angle', Parameter.Type.INTEGER)
        self.declare_parameter('max_scan_angle', Parameter.Type.INTEGER)
        self.declare_parameter('distance_constraint', Parameter.Type.DOUBLE)
        self.declare_parameter('surge_velocity',Parameter.Type.DOUBLE)
        self.declare_parameter('max_yaw_rate',Parameter.Type.DOUBLE)

        # Get parameters
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        enable_search_mode = self.get_parameter('enable_search_mode').get_parameter_value().bool_value
        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.canny_min = self.get_parameter('canny_min_threshold').get_parameter_value().integer_value
        self.canny_max = self.get_parameter('canny_max_threshold').get_parameter_value().integer_value
        msis_vertical_beamwidth = self.get_parameter('msis_vertical_beamwidth').get_parameter_value().integer_value
        self.distance_in_meters = self.get_parameter('standoff_distance_meters').get_parameter_value().double_value
        self.n_points = self.get_parameter('points_to_sample_from_curve').get_parameter_value().integer_value
        self.min_scan_angle = self.get_parameter('min_scan_angle').get_parameter_value().integer_value
        self.max_scan_angle = self.get_parameter('max_scan_angle').get_parameter_value().integer_value
        self.distance_constraint = self.get_parameter('distance_constraint').get_parameter_value().double_value
        self.max_surge = self.get_parameter('surge_velocity').get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value

        if enable_search_mode:
            self.minimum_depth_for_path = -(math.tan(math.radians(msis_vertical_beamwidth / 2)) * self.distance_in_meters)
        else:
            self.minimum_depth_for_path = 0

        # Subscriber
        self.create_subscription(OccupancyGrid, costmap_topic, self.mapCB, 10)

        # Publishers
        self.pub_path = self.create_publisher(Path, path_topic, 10)
        self.best_point_pub = self.create_publisher(Point, f"{path_topic}/best_point", 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, f"{path_topic}/distance_to_obstacle", 10)
        self.image_process_pipeline_pub = self.create_publisher(Image, f"{path_topic}/image", 10)
        self.bridge = CvBridge()
        
        #TF Buffer and Listener
        self.buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.buffer, self)

        #TF BroadCaster
        self.br = tf2_ros.TransformBroadcaster(self)

        self.start_time = time.time()

        self.path = None
        
        #LifeCycle Node Transitions
        target_node = '/alpha_rise/costmap'
        self.client = self.create_client(ChangeState, f'{target_node}/change_state')
        
        self.transition()
    
    def transition(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        label = "TRANSITION"
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Successfully performed transition: {label}')
        else:
            self.get_logger().error(f'Failed to perform transition: {label}')

        time.sleep(1)
        req.transition.id = Transition.TRANSITION_ACTIVATE
        label = "ACTIVATE"
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Successfully performed transition: {label}')
        else:
            self.get_logger().error(f'Failed to perform transition: {label}')
        time.sleep(1)

    def mapCB(self, message:OccupancyGrid):
        """
        Transform Stuff
        """
        # Check robot_localisation has started publishing tf
        # self.buffer.can_transform(self.base_frame, self.odom_frame, rospy.Time(), rospy.Duration(8.0))
        self.buffer.can_transform(self.base_frame, self.odom_frame, Time())
        #Get Odom->Vx TF
        odom_vx_tf = self.buffer.lookup_transform(self.base_frame, self.odom_frame, rclpy.time.Time())
        self.vx_yaw = tf_transform.euler_from_quaternion([odom_vx_tf.transform.rotation.x,
                                            odom_vx_tf.transform.rotation.y,
                                            odom_vx_tf.transform.rotation.z,
                                            odom_vx_tf.transform.rotation.w])[2]
        #Get Vx->Odom TF
        vx_odom_tf = self.buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        self.vx_x = vx_odom_tf.transform.translation.x
        self.vx_y = vx_odom_tf.transform.translation.y
        self.vx_z = vx_odom_tf.transform.translation.z # depth is -ve

        """
        Costmap
        """
        self.width = message.info.width
        self.height = message.info.height
        data = message.data
        self.frame = message.header.frame_id
        self.time = message.header.stamp

        self.resolution = message.info.resolution
        
        """
        Image
        """
        # Costmap to Image
        data = np.array(data).reshape(self.width,self.height)
        data = data.astype(np.uint8)

        # Corrected for Costmap FRAME -> Odom frame.
        data = cv2.flip(data, 1)  
        data = cv2.rotate(data, cv2.ROTATE_90_CLOCKWISE)

        #Erode away speckle noise.
        erode = cv2.erode(data, (5,5),2)
        erode = cv2.erode(erode, (5,5),2)
        # erode = cv2.erode(erode, (5,5),2)

        # Dilate Raw Measurements to get solid reading       
        dilate = cv2.dilate(erode, (5,5), 2)
        dilate = cv2.dilate(dilate, (5,5), 2)
        dilate = cv2.dilate(dilate, (5,5), 2)
        dilate = cv2.dilate(dilate, (5,5), 2)
        dilate = cv2.dilate(dilate, (5,5), 2)

        """
        Edges, Lines and Curves
        """
        #Get Edge
        canny_image = cv2.Canny(dilate,self.canny_min,self.canny_max)
        #Find cordinates of the edges
        raw_pixels = path_utils.find_cordinates_of_max_value(canny_image)
        #Get usable edges
        edge, edge_polar = self.get_usable_edges(raw_pixels)
        #Image and points in Vx_frame
        vx_frame_image, vx_frame_points = self.transform_to_vx_frame(edge_polar)
        #Insert Standoff
        x_list, y_list, edge_frame_debug = self.edge_shifting(vx_frame_image)
        # Fit the line
        path_cells = self.curve_fit(x_list, y_list)
        # Project line in odom frame
        path_odom_frame = self.vx_to_odom_frame_tf(path_cells)
        
        """
        Path
        """
        self.convert_and_publish_path(path_odom_frame)

        
        """
        Image
        """
        if not np.array_equal(vx_frame_image, []) and path_odom_frame != -1:
            vx_frame_image_copy = vx_frame_image.copy()
            ## Compare the Canny and Custom
            compare_path = path_utils.compare_two_lists(raw_pixels, path_odom_frame, self.height, self.width)
            viz_edges = path_utils.compare_two_lists(raw_pixels, edge, self.height, self.width)
            compare_edge = path_utils.compare_points_with_image(vx_frame_image_copy, np.int_(path_cells))

            mix= np.hstack((data, dilate,canny_image, viz_edges, compare_path, compare_edge))
            image_msg = self.bridge.cv2_to_imgmsg(mix)
            self.image_process_pipeline_pub.publish(image_msg)
        
        """
        Distance to Obstacle
        """
        distance_to_obstacle = self.get_distance_to_obstacle(vx_frame_image)
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 1:
            self.start_time = time.time()
            msg = Float32()
            msg.data = float(distance_to_obstacle)
            self.obstacle_distance_pub.publish(msg)

        """
        Visualization
        """
        if self.debug:
            if not np.array_equal(vx_frame_image, []) and path_odom_frame != -1:
                cv2.imshow("window", mix)
                cv2.imshow("edge_frame", edge_frame_debug)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("Empty Costmap", throttle_duration_sec = 3)
    
    def get_usable_edges(self, coordinates:list):
        """
        Use it to find the outer edge from the canny image. 
        The resultant is the cordinates of max likelihood where the obstacle is.

        Args:
            coordinates: [[x,y]] of edges from Image.

        Returns:
            cartesian_coordinates: List of points that are on the outer edge from canny image.
            polar_coordinates: List of points [r, theta] from the center of the image. (Odom frame, vx as origin)
        
        ------>xCOSTMAP IMAGE
        |
        |
        |
       yV
        """
        if coordinates!= None:
            polar_dict = {}

            # Step 1: Shift coordinates to center of the image
            shifted_coordinates = [(x - self.height//2, self.width//2 - y) for x, y in coordinates]
            """
            ------>x COSTMAP IMAGE
            |
            |   ----->x'
            |   |
            |   |
            |   Vy'
            yV
            """
            shifted_coordinates = [(-y,-x) for x, y in shifted_coordinates]
            """
            ------>x COSTMAP IMAGE
            |
            |      ^x
            |      |
            |      |
            |  y<--- VEHICLE IMAGE FRAME
            |   
            yV
            """
            # Step 2: Convert shifted Cartesian coordinates to polar coordinates
            polar_coordinates = [(np.sqrt(x**2 + y**2), round(np.arctan2(y, x),1)) for x, y in shifted_coordinates]
            
            # Step 3: Check for duplicate theta, keep the smallest distance
            for r, theta in polar_coordinates:
                if theta in polar_dict:
                    # If a duplicate theta is encountered, keep the point with the shortest distance
                    if r < polar_dict[theta][0]:
                        polar_dict[theta] = (r, theta)
                else:
                    polar_dict[theta] = (r, theta)
            polar_coordinates = list(polar_dict.values())
            # Step 4: Convert to Cartesian. (x, y)
            cartesian_coordinates = [[round(r * np.cos(theta)), round(r * np.sin(theta))] for r, theta in polar_coordinates]
            """
            ------>x COSTMAP IMAGE
            |
            |      ^x
            |      |
            |      |
            |  y<--- VEHICLE IMAGE FRAME
            |   
            yV
            """
            cartesian_coordinates = [(-y,-x) for x, y in cartesian_coordinates]
            """
            ------>x COSTMAP IMAGE
            |
            |   ----->x'
            |   |
            |   |
            |   Vy'
            yV
            """

            # Step 5: Shift back
            cartesian_coordinates = [[int(x+self.width//2),int(self.height//2 - y),] for x, y in cartesian_coordinates]
            """ 
            ------>xCOSTMAP IMAGE
            |
            |
            |
            yV
            """
            # Step 6: sample //TO-DO; Time based sampling.
            # if len(cartesian_coordinates) > self.width//4:
            #     diff = len(cartesian_coordinates) - self.width//4
            #     for i in range(diff):
            #         cartesian_coordinates.pop(i)
            #
            # POLAR_CORDINATES IS IN VEHICLE FRAME
            # [x,y] [r,theta]
            return cartesian_coordinates, polar_coordinates
        else:
            return None, None
        
    def transform_to_vx_frame(self, coordinates_list:list, radius=1, color=255):
        """
        Viz function with to show vehicle frame. What the vehicle sees is constant in this frame

        Args:
            coordinates_list: List of points in polar system in Odom frame.

        Returns:
            image: Image with points plotted in vehicle frame.
            cartesian_coordinates: list of points in vehicle frame
        """
        if coordinates_list != None:
            yawd_p = [list(t) for t in coordinates_list]

            vx_theta = self.vx_yaw

            for points in yawd_p:
                points[1] = points[1] - np.float64(vx_theta)
                

            # Convert to Cartesian.
            cartesian_coordinates = [[round(r * np.cos(theta)), round(r * np.sin(theta))] for r, theta in yawd_p]
            """
            ------>x COSTMAP IMAGE
            |
            |      ^x
            |      |
            |      |
            |  y<--- VEHICLE FRAME
            |   
            yV
            """
            cartesian_coordinates = [(-y,-x) for x, y in cartesian_coordinates]
            """
            ------>x COSTMAP IMAGE
            |
            |   ----->x'
            |   |
            |   |
            |   Vy'
            yV
            """
            # Shift back to Image frame
            cartesian_coordinates = [[int(x+self.width//2),int(self.height//2 - y)] for x, y in cartesian_coordinates]
            """ 
            ------>xCOSTMAP IMAGE
            |
            |
            |
            yV
            """

            # Create an empty image (numpy array)
            image = np.zeros((self.height, self.width), dtype=np.uint8)
            # Iterate through the
            #  list of coordinates and draw circles
            for coordinates in cartesian_coordinates:
                center = tuple(coordinates)
                cv2.circle(image, center, radius, color, 1)

            # image = cv2.line(image, (0, 3*self.width//4), (200, 3*self.width//4), 100, 1)
            # image = cv2.line(image, (100, 0), (100, 200), 100, 1)

            #Draw Vx frame
            # image = cv2.line(image, (self.width//2, self.height//2), (self.width//2, self.height//2-5), 200, 1)
            # image = cv2.line(image, (self.width//2, self.height//2), (self.width//2-5, self.height//2), 200, 1) 

            return image, cartesian_coordinates
        else:
            return [],[]
    
    def edge_shifting(self, vx_frame:np.array):
        """
        Function to shift the contour towards the vehicle with a stand-off distance

        Args:
            vx_frame: Costmap image relative to the vehicle.

        Returns:
            x_frame_line_list: List of x_cordinates of the shifted contour
            y_frame_line_list: List of y_cordinates of the shifted contour
        """
        if not np.array_equal(vx_frame, []):
            # vx_frame_cropped = vx_frame[ :, self.width//2 :]
            vx_frame_cropped = vx_frame[ :, :]       
                
            """        
            ------>xVEHICLE COSTMAP IMAGE
            |
            |
            |
            yV
            """
            self.new_origin = []
            vx_frame_pixels = path_utils.find_cordinates_of_max_value(vx_frame_cropped)

            #Atleast need 2 points
            if len(vx_frame_pixels) > 2:
                #New origin from list of points.
                x_coordinates, y_coordinates = zip(*vx_frame_pixels)
                
                #Edge frame origin is point with min y in image frame.
                min_y = min(y_coordinates)
                for pair in vx_frame_pixels:
                    # Check if the first element of the pair matches the given x_value
                    if pair[1] == min_y:
                        # Return the corresponding y value
                        min_x = pair[0]
                #New origin
                self.new_origin.append(min_x)
                self.new_origin.append(min_y)
                
                ##VX_FRAME_IMAGE -> EDGE FRAME
                x_edge_frame_list = []
                y_edge_frame_list = []
                for coords in vx_frame_pixels:
                    shifted_x = coords[0] - self.new_origin[0]
                    shifted_y = coords[1] - self.new_origin[1]
                    x_edge_frame_list.append(shifted_x)
                    y_edge_frame_list.append(shifted_y)
                
                #Vehicle cord in edge frame
                vx_x_edge_frame = self.width//2 - self.new_origin[0]
                vx_y_edge_frame = self.height//2 - self.new_origin[1]
                """        
                ------>x VEHICLE COSTMAP IMAGE
                |         ->x PATH FRAME
                |        yV
                |
                yV
                """

                ##LINEAR REGESH
                # self.slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(x_edge_frame_list, 
                                                                                        #   y_edge_frame_list)
                self.slope, intercept = path_utils.calculate_slope(x_edge_frame_list, y_edge_frame_list)

                #Slope Angle (in radian)
                self.angle_from_e_to_l = math.atan(self.slope)
                
                if self.debug:
                    edge_frame_debug = np.zeros((self.height, self.width,3), dtype=np.uint8)
                    lin_regress_debug = [((self.slope) * x ) for x in x_edge_frame_list]

                    for cords in zip(x_edge_frame_list,y_edge_frame_list ):
                        cords = list(cords)
                        cords[0] = cords[0]+self.width//2
                        cords[1] = cords[1]+self.height//2
                        center = tuple(cords)
                        cv2.circle(edge_frame_debug, center, 1, (255,255,255), 1)
                    
                    #draw best fit line
                    for cords in zip(x_edge_frame_list, lin_regress_debug):
                        cords = list(cords)
                        cords[0] = cords[0]+self.width//2
                        cords[1] = round(cords[1]+self.height//2)
                        center = tuple(cords)
                        cv2.circle(edge_frame_debug, center, 1, (0,0,255), 1)

                    #Draw origin of Edge Frame
                    cv2.circle(edge_frame_debug, (self.width//2, 
                                                self.height//2), 1, (0,255,0), 2)
                    
                edge_frame  = [[x, y] for x, y in zip(x_edge_frame_list, y_edge_frame_list)]

                ##EDGE_FRAME -> LINE_FRAME
                x_line_frame_list, y_line_frame_list = path_utils.rotate_points(edge_frame, self.angle_from_e_to_l)
                #Vehicle cords in Line Frame
                vx_x_line_frame, vx_y_line_frame = path_utils.rotate_points([[vx_x_edge_frame, vx_y_edge_frame]], self.angle_from_e_to_l)
                self.vx_line_frame = [vx_x_line_frame, vx_y_line_frame]
                """        
                ------>xCOSTMAP IMAGE
                |         ->x LINE FRAME    
                |        yV
                |
                yV
                """

                #Insert standoff distance depending on the Y_cord of the vehicle in {L}
                distance_in_cells = self.distance_in_meters*1/self.resolution
                
                if round(vx_y_line_frame[0]) > 0.0:
                    y_line_frame_list = [y + distance_in_cells for y in y_line_frame_list]
                else:
                    y_line_frame_list = [y - distance_in_cells for y in y_line_frame_list]
                
            else:
                x_line_frame_list = y_line_frame_list = 0
            
            if self.debug:
                return x_line_frame_list, y_line_frame_list, edge_frame_debug
            else:
                return x_line_frame_list, y_line_frame_list, None
        else:
            return None, None, None 
       
    def curve_fit(self, x_coordinates:list, y_coordinates:list):
        """
        Fit the curve


        Args:
            x_frame_line_list: List of x_cordinates of the shifted contour
            y_frame_line_list: List of y_cordinates of the shifted contour
        
        Returns:
            vx_frame_model: List of cordinates [[x,y]] of the curve in vehicle relative costmap image.
        """
        if x_coordinates != None or y_coordinates != None:
            try:
                #FIT THE CURVE
                p_opt, p_cov = scipy.optimize.curve_fit(path_utils.model_f, x_coordinates, y_coordinates)
                """        
                ------>xCOSTMAP IMAGE
                |         ->x PATH FRAME
                |        yV
                |
                yV
                """
                # print(p_opt)
                a_opt, b_opt, c_opt = p_opt
                
                ##GET CURVE. INdex starts from min(x)
                x_model = np.linspace(min(x_coordinates), max(x_coordinates), self.n_points)
                y_model = path_utils.model_f(x_model, a_opt, b_opt, c_opt)
                
                ##LINE FRAME->EDGE_FRAME
                lingres  = [[x, y] for x, y in zip(x_model, y_model)]
                #Cost 
                best_point = self.get_best_point(lingres)

                x_edge_frame_list, y_edge_frame_list = path_utils.rotate_points(lingres, -self.angle_from_e_to_l)
                x, y = path_utils.rotate_points([best_point], -self.angle_from_e_to_l)
                """        
                ------>xCOSTMAP IMAGE
                |         ->x PATH FRAME
                |        yV
                |
                yV
                """
                ##EDGE_FRAME -> VX_FRAME_IMAGE
                vx_frame_model = []
                edge_frame = [[x, y] for x, y in zip(x_edge_frame_list, y_edge_frame_list)]

                for coords in edge_frame:
                    shifted_x = coords[0] + self.new_origin[0]
                    shifted_y = coords[1] + self.new_origin[1]
                    vx_frame_model.append([round(shifted_x), round(shifted_y)])

                self.best_point = [round(x[0] + self.new_origin[0]), 
                                   round(y[0] + self.new_origin[1])]
                
                
                """        
                ------>xCOSTMAP IMAGE
                |
                |
                |
                yV
                """

                vx_frame_model = [[x, y] for x,y in vx_frame_model]
                return vx_frame_model

            except TypeError:
                self.get_logger().warn("No solution of curve found", throttle_duration_sec = 3)
                return -1
        else:
            return None
        
    def vx_to_odom_frame_tf(self, cart_list:list):
        """
        Transfer it to odom frame.

        
        Args:
            cart_list: List of cordinates [[x,y]] of the curve in vehicle relative costmap image.

        Returns:
            cartesian_coordinates: List of cordinates [[x,y]] in odom relative costmap image.
        """
        if cart_list != None:
            if cart_list != -1:
                cartesian_coordinates = [(x - self.height//2, self.width//2 - y) for x, y in cart_list]
                self.best_point = [self.best_point[0] - self.height//2, self.width//2 - self.best_point[1]]
                
                #edge frame origin cords in image_frame
                self.new_origin = [self.new_origin[0] - self.height//2, self.width//2 - self.new_origin[1]]
                """
                ------>x COSTMAP IMAGE
                |
                |   ----->x'
                |   |
                |   |
                |   Vy'
                yV
                """
                cartesian_coordinates = [(-y,-x) for x, y in cartesian_coordinates]
                self.best_point = [-self.best_point[1], -self.best_point[0]]
                self.new_origin = [-self.new_origin[1], -self.new_origin[0]]
                """
                ------>x COSTMAP IMAGE
                |
                |      ^x
                |      |
                |      |
                |  y<--- VEHICLE FRAME
                |   
                yV
                """
                # Step 2: Convert shifted Cartesian coordinates to polar coordinates
                polar_coordinates = [[np.sqrt(x**2 + y**2), (np.arctan2(y, x))] for x, y in cartesian_coordinates]
                self.best_point = [np.sqrt(self.best_point[0]**2 + self.best_point[1]**2), np.arctan2(self.best_point[1], self.best_point[0])]
                self.new_origin = [np.sqrt(self.new_origin[0]**2 + self.new_origin[1]**2), np.arctan2(self.new_origin[1], self.new_origin[0])]
                
                vx_theta = self.vx_yaw
                
                # if vx_theta > 1.0:
                #     vx_theta = 1.0
                # # Discout vx_yaw
                for points in polar_coordinates:
                    points[1] = (points[1] + np.float64(vx_theta))
                
                self.best_point[1] = self.best_point[1] + np.float64(vx_theta)
                self.new_origin[1] = self.new_origin[1] + np.float64(vx_theta)

                # Convert to Cartesian (x,y)
                cartesian_coordinates = [[round(r * np.cos(theta)), round(r * np.sin(theta))] for r, theta in polar_coordinates]
                self.best_point = [round(self.best_point[0] * np.cos(self.best_point[1])),round(self.best_point[0] * np.sin(self.best_point[1]))]
                self.new_origin = [round(self.new_origin[0] * np.cos(self.new_origin[1])),round(self.new_origin[0] * np.sin(self.new_origin[1]))]
                
                """
                ------>x COSTMAP IMAGE
                |
                |      ^x
                |      |
                |      |
                |  y<--- VEHICLE FRAME
                |   
                yV
                """
                cartesian_coordinates = [(-y,-x) for x, y in cartesian_coordinates]
                self.best_point = [-self.best_point[1], -self.best_point[0]]
                self.new_origin = [-self.new_origin[1], -self.new_origin[0]]

                """
                ------>x COSTMAP IMAGE
                |
                |   ----->x'
                |   |
                |   |
                |   Vy'
                yV
                """
                # Shift back to Image frame
                cartesian_coordinates = [[int(x+self.width//2),int(self.height//2 - y)] for x, y in cartesian_coordinates]
                self.best_point = [int(self.best_point[0] + self.width//2), int(self.height//2 - self.best_point[1])]
                self.new_origin = [int(self.new_origin[0] + self.width//2), int(self.height//2 - self.new_origin[1])]

                """        
                ------>xCOSTMAP IMAGE
                |
                |
                |
                yV
                """
                # cartesian_coordinates = sorted(cartesian_coordinates, key=lambda coord: coord[1], reverse=True)

                return cartesian_coordinates
            else:
                return -1
        else:
            return None
        
    def convert_and_publish_path(self, shifted_coordinates:list):
        """
        Create and send the path message


        Args:
            shifted_coordinates: List of fitted points in odom relative costmap image
        """
        if self.vx_z <= self.minimum_depth_for_path:
            if shifted_coordinates!= None:
                if shifted_coordinates!= -1:
                    # rospy.loginfo_throttle(3, f"Desired depth achieved, Path published")
                    path = Path()
                    path.header.frame_id = self.frame
                    path.header.stamp =  self.time
                
                    for index, cords in enumerate(shifted_coordinates):
                        pose_stamped = PoseStamped()
                        pose_stamped.header.frame_id = self.frame#
                        pose_stamped.header.stamp = self.time
                        # pose_stamped.header.seq = index

                        #IMAGE TO MAP FRAME FIXED TO ODOMs
                        """
                        ------>x COSTMAP IMAGE
                        |
                        |      ^x
                        |      |
                        |      |
                        |  y<--- VEHICLE FRAME
                        |   
                        yV
                        """
                        x = -cords[1]
                        y = -cords[0]

                        x = ((self.width//2 + x) * self.resolution) + self.vx_x
                        y = ((self.height//2 + y)* self.resolution ) + self.vx_y
                        
                        pose_stamped.pose.position.x = x#self.height//4 - cords[1] * self.resolution
                        pose_stamped.pose.position.y = y#self.height//4 - cords[0] * self.resolution 
                        pose_stamped.pose.orientation.x = np.float64(0)
                        pose_stamped.pose.orientation.y = np.float64(0)
                        pose_stamped.pose.orientation.z = np.float64(0)
                        pose_stamped.pose.orientation.w = np.float64(1)
                        path.poses.append(pose_stamped)
                    self.path = path
                    self.pub_path.publish(path)

                    #Pub Best Point
                    self.best_point = [-self.best_point[1], -self.best_point[0]]
                    self.best_point = [((self.width//2 + self.best_point[0]) * self.resolution) + self.vx_x,
                                        ((self.height//2 + self.best_point[1]) * self.resolution) + self.vx_y]
                    msg = Point()
                    msg.x = self.best_point[0]
                    msg.y = self.best_point[1]
                    msg.z = np.float64(0)
                    self.best_point_pub.publish(msg)

                    self.new_origin = [-self.new_origin[1], -self.new_origin[0]]
                    self.new_origin = [((self.width//2 + self.new_origin[0]) * self.resolution) + self.vx_x,
                                        ((self.height//2 + self.new_origin[1]) * self.resolution) + self.vx_y]
                    
                    # Broadcast Odom-> Edge Frame TF
                    odom_costmap_tf = TransformStamped()
                    # odom_costmap_tf.header.stamp = rospy.Time.now()
                    odom_costmap_tf.header.frame_id = 'alpha_rise/odom'
                    odom_costmap_tf.child_frame_id = 'alpha_rise/costmap/edge_frame'
                    odom_costmap_tf.transform.translation.x = self.new_origin[0]
                    odom_costmap_tf.transform.translation.y = self.new_origin[1]
                    odom_costmap_tf.transform.translation.z = np.float64(0)
                    rotation = tf_transform.quaternion_from_euler(np.float64(3.14),np.float64(0),np.float64(-self.vx_yaw-1.57))
                    odom_costmap_tf.transform.rotation.x = rotation[0]
                    odom_costmap_tf.transform.rotation.y = rotation[1]
                    odom_costmap_tf.transform.rotation.z = rotation[2]
                    odom_costmap_tf.transform.rotation.w = rotation[3]
                    self.br.sendTransform(odom_costmap_tf)

                    # Broadcast Edge Frame-> Line Frame TF
                    odom_costmap_tf = TransformStamped()
                    # odom_costmap_tf.header.stamp = rospy.Time.now()
                    odom_costmap_tf.header.frame_id = 'alpha_rise/costmap/edge_frame'
                    odom_costmap_tf.child_frame_id = 'alpha_rise/costmap/line_frame'
                    odom_costmap_tf.transform.translation.x = np.float64(0)
                    odom_costmap_tf.transform.translation.y = np.float64(0)
                    odom_costmap_tf.transform.translation.z = np.float64(0)
                    rotation = tf_transform.quaternion_from_euler(np.float64(0),np.float64(0), np.float64(self.angle_from_e_to_l))
                    odom_costmap_tf.transform.rotation.x = rotation[0]
                    odom_costmap_tf.transform.rotation.y = rotation[1]
                    odom_costmap_tf.transform.rotation.z = rotation[2]
                    odom_costmap_tf.transform.rotation.w = rotation[3]
                    self.br.sendTransform(odom_costmap_tf)
                
                else:
                    if self.path != None:
                        self.pub_path.publish(self.path)
        
            else:
                path = Path()
                path.header.frame_id = self.frame
                path.header.stamp =  self.time
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.frame#
                pose_stamped.header.stamp = self.time
                pose_stamped.pose.position.x = self.vx_x  #self.height//4 - cords[1] * self.resolution
                pose_stamped.pose.position.y = self.vx_y #self.height//4 - cords[0] * self.resolution 
                pose_stamped.pose.orientation.x = np.float64(0)
                pose_stamped.pose.orientation.y = np.float64(0)
                pose_stamped.pose.orientation.z = np.float64(0)
                pose_stamped.pose.orientation.w = np.float64(1)
                path.poses.append(pose_stamped)
                self.pub_path.publish(path)
        else: # depth greater than -2
            # rospy.loginfo_throttle(3, f"Desired depth: {round(-(math.tan(math.radians(12.5)) * self.distance_in_meters))}, Current Depth: {round(self.vx_z)}")
            path = Path()
            path.header.frame_id = self.frame
            path.header.stamp =  self.time
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame#
            pose_stamped.header.stamp = self.time
            pose_stamped.pose.position.x = self.vx_x  #self.height//4 - cords[1] * self.resolution
            pose_stamped.pose.position.y = self.vx_y #self.height//4 - cords[0] * self.resolution 
            pose_stamped.pose.orientation.x = np.float64(0)
            pose_stamped.pose.orientation.y = np.float64(0)
            pose_stamped.pose.orientation.z = np.float64(0)
            pose_stamped.pose.orientation.w = np.float64(1)
            path.poses.append(pose_stamped)
            self.pub_path.publish(path)
    
    def get_distance_to_obstacle(self, vx_image:np.array):
        """
        Get distance to the obstacle from the vehicle.

        Args:
            vx_image: The vehicle relative costmap image.
        """
        if not np.array_equal(vx_image, []):
            min_y = float('inf')
            shortest_point = None
            raw_pixels = path_utils.find_cordinates_of_max_value(vx_image)
            shifted_coordinates = [(x - self.height//2, self.width//2 - y) for x, y in raw_pixels]
            """
            ------>x COSTMAP IMAGE
            |
            |   ----->x'
            |   |
            |   |
            |   Vy'
            yV
            """
            shifted_coordinates = [(-y,-x) for x, y in shifted_coordinates]
            """
            ------>x COSTMAP IMAGE
            |
            |      ^x
            |      |
            |      |
            |  y<--- VEHICLE FRAME
            |   
            yV
            """
            image_polar = [(np.sqrt(x**2 + y**2), np.degrees(np.arctan2(y, x))) for x,y in shifted_coordinates]
            for r,theta in image_polar:
                if r < min_y:
                    min_y = r
                    shortest_point = [r,theta]
            if shortest_point != None:
                shortest_point_cartesian = shortest_point[0]  * self.resolution
                return shortest_point_cartesian
        else:
            return -1

    def get_best_point(self, line_frame_points):
        """
        Function to calc the best point to pursue.

        Args:
            line_frame_points: list of cordinates [[x,y]] in {L} frame.
        """

        #Cost calc in {L}
        valid_point, valid_distance, valid_angle, valid_track = [],[],[], []
        for index, point in enumerate((line_frame_points)):
            #Get distance to all points from vx{L}
            vehicle_to_point_distance = math.sqrt((point[0] - self.vx_line_frame[0])**2 + (point[1] - self.vx_line_frame[1])**2)

            #Get angle to all points from vx in{L}
            delta_x = point[0] - self.vx_line_frame[0]
            delta_y = point[1] - self.vx_line_frame[1]

            vehicle_to_point_angle = math.atan2(delta_y, delta_x)
            #If vx Y in {L} is +ve, add -180 
            # [[x],[y]]
            if self.vx_line_frame[1][0] > 0.0:
                vehicle_to_point_angle = path_utils.sum_angles_radians(vehicle_to_point_angle, -math.pi)

            #Get track angle in {L}. Len() returns natural numbers. Index starts from 0.
            if index < len(line_frame_points) -2:
                next_point = line_frame_points[index+1]
                track_angle = math.atan2((next_point[1] - point[1]), 
                                         (next_point[0] - point[0]))
            else:
                track_angle = 0
            
            vehicle_to_point_angle_degree = math.degrees(vehicle_to_point_angle)    
            #Heading constraint.
            if self.min_scan_angle < (vehicle_to_point_angle_degree) and (vehicle_to_point_angle_degree) < self.max_scan_angle:
                #Distance constraint.
                if vehicle_to_point_distance > self.distance_constraint * 1/self.resolution:
                    valid_distance.append(vehicle_to_point_distance)
                    valid_angle.append(vehicle_to_point_angle)
                    valid_track.append(track_angle)
                    valid_point.append(point)

        #Get Cost.
        lin_component = [distance/self.max_surge for distance in valid_distance]
        yaw_component = [abs(path_utils.sum_angles_radians(track, -angle))/self.max_yaw_rate for angle, track in zip(valid_angle,valid_track)]
        cost = [lin + ang for lin, ang in zip(lin_component, yaw_component)]
        
        try:
            #Get the point with min cost.
            best_point = min(cost)
            min_index = cost.index(best_point)
            # print(f"Distance: {valid_distance[min_index]}, Angle of (L): {math.degrees(self.angle_from_e_to_l)}, Angle of point: {math.degrees(valid_angle[min_index])}")
            return valid_point[min_index]

        except ValueError:
            # print("No valid point")
            return -1

def main():
    rclpy.init()
    node = PathGen()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()