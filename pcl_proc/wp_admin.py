#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Manages the autonomy state machine of the vehicle
#tony.jacob@uri.edu

#rosbag record /alpha_rise/path/state /alpha_rise/path/distance_to_obstacle /alpha_rise/odometry/filtered/local
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped, Point
from std_msgs.msg import Float32, Int16
import math
from mvp_msgs.srv import  GetState, ChangeState, GetWaypoints
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
        self.depth = self.get_parameter('operating_depth').get_parameter_value().double_value

        #To remove surface reflections from FLS, this is the min depth, the vehicle must be at.
        # self.depth = -math.tan(math.radians(self.depth)) * self.standoff_distance_in_meters

        self.declare_parameter('search_mode_initial_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('search_mode_timer', Parameter.Type.INTEGER)
        self.declare_parameter('follow_mode_timer',Parameter.Type.INTEGER)
        self.declare_parameter('exit_mode_distance',Parameter.Type.DOUBLE)

        # Read parameters
        self.search_mode_initial_radius = self.get_parameter('search_mode_initial_radius').get_parameter_value().double_value

        # Timers
        self.search_mode_timer_param = self.get_parameter('search_mode_timer').get_parameter_value().integer_value
        self.search_mode_timer = time.time()

        self.follow_mode_timer_param = self.get_parameter('follow_mode_timer').get_parameter_value().integer_value
        self.follow_flag = 0

        self.exit_mode_distance = self.get_parameter('exit_mode_distance').get_parameter_value().double_value

        # Declare publishers
        self.pub_update = self.create_publisher(PolygonStamped, update_waypoint_topic, 1)
        self.pub_state = self.create_publisher(Int16, path_topic + '/state', 1)

        # Declare subscribers
        self.create_subscription(Path, path_topic, self.path_cB, 1)
        self.create_subscription(Float32, path_topic + '/distance_to_obstacle', self.distance_cB, 1)
        self.create_subscription(Point, path_topic + "/best_point", self.point_cB, 1)

        # Declare services
        self.get_waypoint_service_client = self.create_client(GetWaypoints, self.get_waypoint_service_name)
        self.get_state_service_client = self.create_client(GetState, self.get_state_service_name)
        self.change_state_service_client = self.create_client(ChangeState, self.change_state_service_name)

        
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
        
        self.bool_search_mode = False

        self.bool_exit_mode = False

    def point_cB(self, msg):
        """
        Best point callback.
        """
        self.x, self.y, z = msg.x, msg.y, msg.z

    def distance_cB(self, msg):
        """
        Distance to obstacle callback. 
        This function syncs the distance with the autonomy state machine indicator.
        Depending on the waypoints in the Helm, 
        One can determine if in Following (0) or IceReac (1)
        """
        self.distance_to_obstacle = msg.data
        
        request = GetWaypoints.Request()
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
                    # print(self.distance_to_obstacle,"reacq",time.time())
                    msg.data = 1
                    self.pub_state.publish(msg) 

    def path_cB(self, msg):
        """
        Path topic callback.
        Depending on the number & value of points and state of the autonomy;
        The behaviours are implemented.
        """

        #Vx position and bearing in Odom frame.
        self.base_to_odom_tf = self.tf_buffer.lookup_transform("alpha_rise/odom", "alpha_rise/base_link", 
                                                              rclpy.time.Time())
        
        #Odom frame point in Vx frame
        self.odom_to_base_tf = self.tf_buffer.lookup_transform("alpha_rise/base_link", "alpha_rise/odom", 
                                                               rclpy.time.Time())
        #Create Waypoint Message
        wp = PolygonStamped()
        wp.header.stamp = msg.header.stamp
        wp.header.frame_id = msg.header.frame_id
        
        # Valid path is n_points long. 
        # Path is always being published.
        # If same path, then no new path is then published.
        if len(msg.poses) == self.n_points:
            
            #Check number of points in front of the vehicle
            n_points_above_vx = 0
            for pose in msg.poses:
                path_vx_frame = tf2_geometry_msgs.do_transform_pose_stamped(pose, self.odom_to_base_tf)
                if path_vx_frame.pose.position.x > 0:
                    n_points_above_vx += 1 

            if self.state == "survey":

                if msg.poses != self.poses:
                    
                    #grab time of follow_mode initializing
                    if self.follow_flag == 0:
                        self.follow_mode_timer = time.time()
                        self.follow_flag =+ 1
                    
                    #Feed best point.
                    if(time.time() - self.follow_mode_timer) < self.follow_mode_timer_param:
                        self.get_logger().info("Following Mode", throttle_duration_sec = 3)
                        self.bool_search_mode = False
                        best_point = Point32()
                        best_point.x = self.x
                        best_point.y = self.y
                        best_point.z = self.depth
                        wp.polygon.points.append(best_point)
                        self.pub_update.publish(wp)
                        self.poses = msg.poses
                    
                    #Chart a course away from the iceberg when timer runs out.
                    #Go to a point 90 degree port side of Vx
                    else:
                        self.get_logger().info(f"Exit sequence. Timer ran out at {self.follow_mode_timer_param}s")
                        self.exit_sequence(wp)

            #Iceberg Reacquisition Mode is when 
            #the vehicle reaches end of a valid path.
            elif n_points_above_vx <= 3: #self.state == "start":     
                self.iceberg_reacquisition_mode(wp)
        
        #Path is still published when no costmap. But the n_points is 1 (vx_x, vx_y)
        #We use that parameter to create a new bhvr mode.
        else:
            # rospy.loginfo("Searching Mode")
            if self.state == "start":
                self.count_concentric_circles += 1
                self.searching_mode(wp)

            elif self.state == "survey":
                #If timer runs out, then the node is killed.
                if(time.time() - self.search_mode_timer) > self.search_mode_timer_param: #sec
                    service_client_change_state = self.create_client(ChangeState,self.change_state_service)
                    request = ChangeState.Request("start", self.node_name)
                    response = service_client_change_state.call_async(request)
                    self.get_logger().warn("Search Mode Took too long --shutting down")
                    self.destroy_node()
                    rclpy.shutdown()
        
    def exit_sequence(self, wp):
        """
        Function to navigate the vehicle 
        away from the iceberg when the timer runs out.
        """
        self.bool_exit_mode = True
                
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
        
        exit_msg = Point32()
        exit_msg.x = exit_point_odom_frame.point.x
        exit_msg.y = exit_point_odom_frame.point.y
        exit_msg.z = np.float64(0)

        wp.polygon.points.append(exit_msg)
        self.pub_update.publish(wp)
        time.sleep(10)
        self.get_logger().info(
                    f"Follow Mode completed. Timeout of {self.follow_mode_timer_param}s. "
                    f"Starting course away from the iceberg to {exit_point_odom_frame.point}"
                    )
        self.destroy_node()
        rclpy.shutdown()

    def searching_mode(self, wp):
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
            msg = Point32()
            msg.x = search_mode_points[i].point.x
            msg.y = search_mode_points[i].point.y
            msg.z = search_mode_depth
            wp.polygon.points.append(msg)

        request = ChangeState.Request()
        request.state = "survey"
        request.caller = self.node_name
        future = self.change_state_service_client.call_async(request)
        future.add_done_callback(self.get_state_callback)
        self.pub_update.publish(wp)
        self.get_logger().info("Search Mode", throttle_duration_sec = 3)
        time.sleep(1)

    def iceberg_reacquisition_mode(self, wp):
        """
        Function to navigate the vehicle 
        so as to reacquire acoustic contact
        """
        #Switch state to survey_3d
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
            msg =Point32()
            msg.x = corner_bhvr_points[i].point.x 
            msg.y = corner_bhvr_points[i].point.y
            msg.z = self.depth
            wp.polygon.points.append(msg)
        # rospy.loginfo_throttle(3,"Iceberg Reacquisition Mode")
        self.get_logger().info("Iceberg Reacquisition Mode", throttle_duration_sec = 3)
        self.pub_update.publish(wp)

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

    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.spin()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()