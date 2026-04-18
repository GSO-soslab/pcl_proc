#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Manages the autonomy state machine of the vehicle
#tony.jacob@uri.edu

#ros2 bag record /alpha_rise/path/state /alpha_rise/path/distance_to_obstacle /alpha_rise/odometry/filtered/local /alpha_rise/path /alpha_rise/fls/pointcloud
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float32, Int16
from std_srvs.srv import SetBool
import math
from mvp_msgs.srv import  GetState, ChangeState, GetWaypoints, SetString
from mvp_msgs.msg import Waypoints, Waypoint
import time
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class Wp_Admin(Node):
    def __init__(self):
        super().__init__('waypoint_admin')
        """
        Constructer. Init all pubs, subs, variables
        """

        self.declare_parameter("stand_off_distance", Parameter.Type.DOUBLE)
        self.declare_parameter('update_waypoint_topic', Parameter.Type.STRING)
        self.declare_parameter('path_topic', Parameter.Type.STRING)
        self.declare_parameter('get_state_service', Parameter.Type.STRING)
        self.declare_parameter('change_state_service', Parameter.Type.STRING )
        self.declare_parameter('get_waypoint_service', Parameter.Type.STRING)
        self.declare_parameter('n_points', Parameter.Type.INTEGER)
        self.declare_parameter('reacquisition_s_param', Parameter.Type.DOUBLE)
        self.declare_parameter('check_state_update_rate', Parameter.Type.INTEGER)
        self.declare_parameter('operating_depth', Parameter.Type.DOUBLE)
        
        self.standoff_distance_in_meters = self.get_parameter("stand_off_distance").get_parameter_value().double_value
        update_waypoint_topic = self.get_parameter('update_waypoint_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.get_state_service_name = self.get_parameter('get_state_service').get_parameter_value().string_value
        self.change_state_service_name = self.get_parameter('change_state_service').get_parameter_value().string_value
        self.get_waypoint_service_name = self.get_parameter('get_waypoint_service').get_parameter_value().string_value
        self.n_points = self.get_parameter('n_points').get_parameter_value().integer_value
        self.reacquisition_s_param = self.get_parameter('reacquisition_s_param').get_parameter_value().double_value
        self.update_rate = self.get_parameter('check_state_update_rate').get_parameter_value().integer_value
        self.clear_entire_costmap_name = "/alpha_rise/clear_entirely_costmap"

        #To remove surface reflections from FLS, this is the min depth, the vehicle must be at.
        # self.depth = -math.tan(math.radians(self.depth)) * self.standoff_distance_in_meters

        self.declare_parameter('search_mode_initial_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('search_mode_timer', Parameter.Type.INTEGER)
        self.declare_parameter('follow_mode_timer',Parameter.Type.INTEGER)
        self.declare_parameter('exit_mode_distance',Parameter.Type.DOUBLE)
        self.declare_parameter('follow_mode_surge', Parameter.Type.DOUBLE)
        self.declare_parameter('reacquisition_surge', Parameter.Type.DOUBLE)
        self.declare_parameter('search_mode_surge', Parameter.Type.DOUBLE)
        self.declare_parameter('exit_mode_surge', Parameter.Type.DOUBLE)


        # Read parameters
        self.search_mode_initial_radius = self.get_parameter('search_mode_initial_radius').get_parameter_value().double_value

        # Timers
        self.search_mode_timer_param = self.get_parameter('search_mode_timer').get_parameter_value().integer_value
        self.search_mode_timer = time.time()

        self.follow_mode_timer_param = self.get_parameter('follow_mode_timer').get_parameter_value().integer_value
        self.follow_flag = 0

        self.exit_mode_distance = self.get_parameter('exit_mode_distance').get_parameter_value().double_value

        # Surge Params
        self.follow_mode_surge = self.get_parameter('follow_mode_surge').get_parameter_value().double_value
        self.reacquisition_surge = self.get_parameter('reacquisition_surge').get_parameter_value().double_value
        self.exit_mode_surge = self.get_parameter('exit_mode_surge').get_parameter_value().double_value
        self.search_mode_surge = self.get_parameter('search_mode_surge').get_parameter_value().double_value
        
        # Declare publishers
        self.pub_update = self.create_publisher(Waypoints, update_waypoint_topic, 1)
        self.pub_state = self.create_publisher(Int16, path_topic + '/state', 1)

        # Declare subscribers
        self.create_subscription(Path, path_topic, self.path_cB, 1)
        self.create_subscription(Float32, path_topic + '/distance_to_obstacle', self.distance_cB, 1)
        self.create_subscription(Point, path_topic + "/best_point", self.point_cB, 1)

        self.create_subscription(Int16, path_topic + "/surge", self.surge_cB, 1)


        # Declare services
        self.get_waypoint_service_client = self.create_client(GetWaypoints, self.get_waypoint_service_name)
        self.get_state_service_client = self.create_client(GetState, self.get_state_service_name)
        self.change_state_service_client = self.create_client(ChangeState, self.change_state_service_name)
        self.depth_planner_service_client = self.create_client(SetBool, '/alpha_rise/iceberg/plan_depth')
        
        self.create_service(SetBool, '/alpha_rise/iceberg/revisit', self.revisit_service_cb)
        self.create_service(SetString, '/alpha_rise/mission', self.mission_service_cb)

        
        #Declare variables
        self.state = None
        self.distance_to_obstacle = None
        self.create_timer(self.update_rate, self.check_state)
        
        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer,self)

        self.node_name = self.get_name()

        self.poses = []

        self.count_concentric_circles = 0
        self.x, self.y = 0.0,0.0
        
        #Number of loops
        self.loop = 0

        self.plan_depth = False

        self.bool_search_mode = False

        self.valid_point = True

        self.valid_best_point = True

        self.bool_exit_mode = False

        self.mission_command = "EMPTY"

        self.get_logger().info("Administrator launched. Use /alpha_rise/mission service to engage. START, RESTART or CONTINUE")

    def mission_service_cb(self, request, response):
        """
        This service indicates whether to start, restart or continue
        the mapping
        """
        if request.data in ["START", "RESTART", "CONTINUE"]:
            self.mission_command = request.data
            response.success = True
            response.message = f"VALID CMD RECIEVED, EXECUTING MISSION"
            self.bool_exit_mode = False

            if self.mission_command == "CONTINUE":
                request = SetBool.Request()
                request.data = True
                future = self.depth_planner_service_client.call_async(request)
                future.add_done_callback(self.depth_planner_callback)   
        else:
            response.success = False
            response.message = f"INVALID CMD"
        return response
    
    def depth_planner_callback(self,future):
        response = future.result()

    def revisit_service_cb(self, request, response):
        """
        On this service call, either by loop detection or timeout, 
        go to exit mode
        """
        wpts = Waypoints()
        if request.data:
            response.success = True
            response.message = "Revisit triggered."
            self.loop += 1
        else:
            response.success = False
            response.message = "Revisit not triggered"
       
        print(response.message, flush=True)
        self.exit_mode(wpts,info= response.message)
        return response
    
    def surge_cB(self, msg):
        if msg.data == 1:
            self.plan_depth = True
        else:
            self.plan_depth = False

    def point_cB(self, msg):
        """
        Best point callback.
        """
        vx = round(self.base_to_odom_tf.transform.translation.x)
        vy = round(self.base_to_odom_tf.transform.translation.y)

        if math.hypot(round(vx-msg.x), round(vy-msg.y)) > 5.0:
            self.x, self.y, z = msg.x, msg.y, msg.z
            self.valid_best_point = True
        else:
            self.valid_best_point =False

    def distance_cB(self, msg):
        """
        Distance to obstacle callback. 
        This function syncs the distance with the autonomy state machine indicator.
        Depending on the waypoints in the Helm, 
        One can determine if in Following (0) or IceReac (1)
        """
        self.distance_to_obstacle = msg.data
        
        request = GetWaypoints.Request()
        request.count.data = 0
        future = self.get_waypoint_service_client.call_async(request)
        future.add_done_callback(self.get_n_waypoints)

    def get_n_waypoints(self, future):
        if future.done():
            response = future.result()
            n_wpt = len(response.wpt)
            msg = Int16()
            # 2 is the len(response.wpt) when in following.
            if n_wpt == 2:
                if self.bool_exit_mode:
                    # print(self.distance_to_obstacle,"exit_mode",time.time())
                    msg.data=2
                    self.pub_state.publish(msg)
                else:
                    # print(self.distance_to_obstacle,"Following",time.time())
                    msg.data=0
                    self.pub_state.publish(msg)
            #Search Mode
            else:
                if self.bool_search_mode:
                    # print(self.distance_to_obstacle, "search",time.time())
                    msg.data = -1
                    self.pub_state.publish(msg)
                #Iceberg Reaaq
                else:
                    if self.mission_command != "EMPTY":
                    # print(self.distance_to_obstacle,"reacq",time.time())
                        msg.data = 1
                        self.pub_state.publish(msg)
                    else:
                        msg.data = -2
                        self.pub_state.publish(msg) 

    def path_cB(self, msg):
        """
        Path topic callback.
        Depending on the number & value of points and state of the autonomy;
        The behaviours are implemented.
        """
        if self.mission_command == "START" or self.mission_command == "RESTART":
            self.depth = self.get_parameter('operating_depth').get_parameter_value().double_value
            self.mission_core(msg)
        
        elif self.mission_command == "CONTINUE":
            self.depth = (self.loop+1)*self.get_parameter('operating_depth').get_parameter_value().double_value
            self.mission_core(msg)
        
    def mission_core(self, msg):
        #Vx position and bearing in Odom frame.
        self.base_to_odom_tf = self.tf_buffer.lookup_transform("alpha_rise/odom", "alpha_rise/base_link", 
                                                            rclpy.time.Time())
        
        #Odom frame point in Vx frame
        self.odom_to_base_tf = self.tf_buffer.lookup_transform("alpha_rise/base_link", "alpha_rise/odom", 
                                                            rclpy.time.Time())
        #Create Waypoint Message
        wpts = Waypoints()
        self.header = msg.header
        # Valid path is n_points long. 
        # Path is always being published.
        # If same path, then no new path is then published.
        if len(msg.poses) == self.n_points:

            # Get terminal poses
            start_pose = msg.poses[0]
            end_pose   = msg.poses[-1]

            # Distances to best_point position
            dx_start = start_pose.pose.position.x - self.x
            dy_start = start_pose.pose.position.y - self.y
            dist_start = math.hypot(dx_start, dy_start)

            dx_end = end_pose.pose.position.x - self.x
            dy_end = end_pose.pose.position.y - self.y
            dist_end = math.hypot(dx_end, dy_end)

            # Select closest terminal point
            if dist_start <= dist_end:
                self.closest_terminal_pose = start_pose
            else:
                self.closest_terminal_pose = end_pose
            
            vx = round(self.base_to_odom_tf.transform.translation.x)
            vy = round(self.base_to_odom_tf.transform.translation.y)
            if math.hypot(round(vx-self.closest_terminal_pose.pose.position.x), round(vy-self.closest_terminal_pose.pose.position.y)) > 10.0:
                valid_closest_point = True
            else:
                valid_closest_point = False
                
            if valid_closest_point and self.valid_best_point:
                self.valid_point = True
            else:
                self.valid_point = False
                
            if self.state == "survey":
                if self.valid_point:

                    self.search_mode_timer = time.time()
                    #grab time of follow_mode initializing
                    if self.follow_flag == 0:
                        self.follow_mode_timer = time.time()
                        self.follow_flag =+ 1
                    
                    self.get_logger().info(f"Following Mode in {self.state} with {round(self.follow_mode_timer_param - (time.time() - self.follow_mode_timer))}s remaining", throttle_duration_sec = 15)

                    #Feed best point.
                    if(time.time() - self.follow_mode_timer) < self.follow_mode_timer_param:
                        self.bool_search_mode = False
                        
                        wpt = Waypoint()
                        wpt.header = msg.header
                        
                        if self.plan_depth:
                            wpt.u = 0.1
                        else:
                            wpt.u = self.follow_mode_surge

                        best_point = Point()
                        best_point.x = self.x
                        best_point.y = self.y
                        best_point.z = self.depth
                        wpt.wpt = best_point
                        wpts.wpt.append(wpt)
                        # wp.polygon.points.append(best_point)

                        wpt = Waypoint()
                        wpt.header = msg.header
                        wpt.u = self.follow_mode_surge
                        best_point = Point()    
                        best_point.x = self.closest_terminal_pose.pose.position.x
                        best_point.y = self.closest_terminal_pose.pose.position.y
                        best_point.z = self.depth
                        wpt.wpt = best_point
                        wpts.wpt.append(wpt)
                        self.pub_update.publish(wpts)

                        # self.poses = msg.poses            
                    #Chart a course away from the iceberg when timer runs out.
                    #Go to a point 90 degree port side of Vx
                    else:
                        self.get_logger().info(f"Exit sequence. Timer ran out at {self.follow_mode_timer_param}s")
                        self.exit_mode(wpts, info = f"Mission completed. Timeout of {self.follow_mode_timer_param}s.")
                    # else:
                    #     self.iceberg_reacquisition_mode(wpts)
            #Iceberg Reacquisition Mode is when 
            #the vehicle reaches end of a valid path.
            elif self.state == "start":     
                self.iceberg_reacquisition_mode(wpts)
        
        #Path is still published when no costmap. But the n_points is 1 (vx_x, vx_y)
        #We use that parameter to create a new bhvr mode.
        else:
            # rospy.loginfo("Searching Mode")
            if self.state == "start":
                self.count_concentric_circles += 1
                self.search_mode(wpts)

            elif self.state == "survey":
                #If timer runs out, then the node is killed.
                if(time.time() - self.search_mode_timer) > self.search_mode_timer_param: #sec
                    request = ChangeState.Request()
                    request.state = "start"
                    request.caller = self.node_name
                    future = self.change_state_service_client.call_async(request)
                    future.add_done_callback(self.get_state_callback)

                    self.get_logger().warn("Search Mode Took too long --shutting down")
                    self.destroy_node()
                    rclpy.shutdown()
            
    def exit_mode(self, wpts, info):
        """
        Function to navigate the vehicle 
        away from the iceberg when the timer runs out.
        """
        self.bool_exit_mode = True
        # Costmap frames only exists if path can be generated. If in Reacquisition mode,
        # then direct away from the iceberg
        if self.tf_buffer.can_transform("alpha_rise/odom",
                                        "alpha_rise/costmap/line_frame",
                                        rclpy.time.Time() ) and self.tf_buffer.can_transform("alpha_rise/costmap/line_frame",
                                                                                "alpha_rise/base_link",
                                                                                rclpy.time.Time()  # latest available
                                                                                ):
            #Line_frame point in Odom Frame
            line_frame_to_odom_tf = self.tf_buffer.lookup_transform("alpha_rise/odom", 
                                                                    "alpha_rise/costmap/line_frame", 
                                                                rclpy.time.Time())
            
            vx_to_line_frame_tf = self.tf_buffer.lookup_transform("alpha_rise/costmap/line_frame", 
                                                                    "alpha_rise/base_link", 
                                                                rclpy.time.Time())
            exit_point = PointStamped()
            exit_point.point.x = np.float64(0)
            
            if vx_to_line_frame_tf.transform.translation.y > 0:
                exit_point.point.y = self.exit_mode_distance + self.standoff_distance_in_meters
            else:
                exit_point.point.y = -self.exit_mode_distance - self.standoff_distance_in_meters
                
            exit_point_odom_frame = tf2_geometry_msgs.do_transform_point(exit_point, line_frame_to_odom_tf)
            
            exit_msg = Point()
            exit_msg.x = exit_point_odom_frame.point.x
            exit_msg.y = exit_point_odom_frame.point.y
            exit_msg.z = np.float64(0)
        
        else:
            #Point in BaseFrame
            exit_point = PointStamped()
            exit_point.point.x = np.float64(0)
            exit_point.point.y = self.exit_mode_distance
            
            #Transform to Odom
            exit_point_odom_frame = tf2_geometry_msgs.do_transform_point(exit_point, self.base_to_odom_tf)

            exit_msg = Point()
            exit_msg.x = exit_point_odom_frame.point.x
            exit_msg.y = exit_point_odom_frame.point.y
            exit_msg.z = np.float64(0)
        
        wpt = Waypoint()
        wpt.header = self.header
        wpt.wpt = exit_msg
        wpt.u = self.exit_mode_surge
        wpts.wpt.append(wpt)
        self.pub_update.publish(wpts)

        msg = Int16()
        msg.data=2
        self.pub_state.publish(msg)
        self.get_logger().info(info)
        time.sleep(10)
        self.mission_command = "EMPTY"
        self.count_concentric_circles = 0
        self.bool_search_mode = False

    def search_mode(self, wpts):
        self.bool_search_mode = True

        #Change here for initial depth.
        search_mode_depth = round(-(math.tan(math.radians(12.5)) * 50),2)
        """
        Function to navigate the vehicle 
        to start searching for iceberg at depth.
        """

        if self.count_concentric_circles == 1:
            #Get vehicle position in Odom
            self.search_mode_center = PointStamped()
            self.search_mode_center.point.x = self.base_to_odom_tf.transform.translation.x
            self.search_mode_center.point.y = self.base_to_odom_tf.transform.translation.y

        #Transform to current vx_frame
        center_in_vx_frame = tf2_geometry_msgs.do_transform_point(self.search_mode_center, self.odom_to_base_tf)        
        
        #Grow the search radius incrementally.
        search_mode_radius = self.search_mode_initial_radius * self.count_concentric_circles
        search_mode_points = self.draw_arc(number_of_points=self.n_points, 
                                            start_angle=0, 
                                            end_angle=2*math.pi,
                                            center=[center_in_vx_frame.point.x, center_in_vx_frame.point.y],
                                            radius = search_mode_radius)
    
        for i in range(len(search_mode_points)):
            wpt = Waypoint()
            wpt.header = self.header
            msg = Point()
            msg.x = search_mode_points[i].point.x
            msg.y = search_mode_points[i].point.y
            msg.z = search_mode_depth
            wpt.wpt = msg
            wpt.u = self.search_mode_surge
            wpts.wpt.append(wpt)

        request = ChangeState.Request()
        request.state = "survey"
        request.caller = self.node_name
        future = self.change_state_service_client.call_async(request)
        future.add_done_callback(self.get_state_callback)
        self.pub_update.publish(wpts)
        self.get_logger().info("Search Mode", throttle_duration_sec = 3)
        self.state = "survey"
        time.sleep(1)

    def iceberg_reacquisition_mode(self, wpts):
        """
        Function to navigate the vehicle 
        so as to reacquire acoustic contact
        """
        #Switch state to survey_3d
        if self.state == "start":
            request = ChangeState.Request()
            request.state = "survey"
            request.caller = self.node_name
            future = self.change_state_service_client.call_async(request)
            future.add_done_callback(self.get_state_callback)
        #The center point of the circle in vx frame
        point_of_obstacle = [self.reacquisition_s_param*self.standoff_distance_in_meters, -self.standoff_distance_in_meters]
        corner_bhvr_points = self.draw_arc(number_of_points=self.n_points, 
                                                   start_angle=math.pi/2, 
                                                   end_angle=0,
                                                   center=point_of_obstacle,
                                                   radius = self.standoff_distance_in_meters)
    
        #Append the waypoints
        for i in range(len(corner_bhvr_points)):
            wpt = Waypoint()
            wpt.header = self.header
            wpt.u = self.reacquisition_surge
            
            msg = Point()
            msg.x = corner_bhvr_points[i].point.x 
            msg.y = corner_bhvr_points[i].point.y
            msg.z = self.depth
            wpt.wpt = msg
      
            wpts.wpt.append(wpt)
        self.get_logger().info("Iceberg Reacquisition Mode", throttle_duration_sec = 3)
        self.pub_update.publish(wpts)


    def reaquisition_points(self):
        x0 = self.closest_terminal_pose.pose.position.x
        y0 = self.closest_terminal_pose.pose.position.y

        # Circle radius
        r = self.standoff_distance_in_meters  # meters, adjust as needed

        # Center for clockwise arc (example: center below starting point)
        xc = x0
        yc = y0 - r

        # Generate points along arc (45 degrees)
        num_points = 10
        theta_start = math.pi/2   # start angle relative to center
        theta_end   = math.pi/2 - math.pi/4  # clockwise 45 degrees
        arc_points = []

        for i in range(num_points + 1):
            theta = theta_start + (theta_end - theta_start) * i / num_points
            x = xc + r * math.cos(theta)
            y = yc + r * math.sin(theta)
            arc_points.append((x, y))

        return arc_points
    
    def check_state(self):
        """
        Function to check the state of the helm
        """
        request = GetState.Request()
        future = self.get_state_service_client.call_async(request)
        future.add_done_callback(self.get_state_callback)


    def get_state_callback(self,future):
        response = future.result()
        self.state = response.state.name
        
        
    def extend_line_from_point(self, point, orientation, length):
        # Convert orientation from degrees to radians
        angle_radians = math.radians(orientation)

        # Calculate the coordinates of the second point
        x2 = point[0] + length * math.cos(angle_radians)
        y2 = point[1] + length * math.sin(angle_radians)

        return (x2, y2)

    def draw_arc(self, number_of_points, start_angle, end_angle, center, radius):
        """
        Create an arc in vehicle frame using parametric equation of circle.
        Then transform to odom.
        """
        #init lists
        corner_bhvr_points = []
        points_in_odom_frame = []
        
        #list of angle increments
        angles = np.linspace(start_angle, end_angle, number_of_points)

        #list of circle points in vx_frame
        corner_bhvr_points = [(center[0] + radius * math.cos(angle), 
                            center[1] + radius * math.sin(angle))
                            for angle in angles]
        
        #Tf to odom frame
        for points in corner_bhvr_points:
            point_msg = PointStamped()
            point_msg.point.x = points[0]
            point_msg.point.y = points[1]
            point_in_odom_frame = tf2_geometry_msgs.do_transform_point(point_msg, self.base_to_odom_tf)
            points_in_odom_frame.append(point_in_odom_frame)

        return points_in_odom_frame

def main():
    rclpy.init()
    node = Wp_Admin()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()