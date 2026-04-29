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

        self.declare_parameter('odom_frame', Parameter.Type.STRING)
        self.declare_parameter('base_frame', Parameter.Type.STRING)
        self.declare_parameter('line_frame', Parameter.Type.STRING)
        self.declare_parameter('edge_frame', Parameter.Type.STRING)
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
        
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.line_frame = self.get_parameter('line_frame').get_parameter_value().string_value
        self.edge_frame = self.get_parameter('edge_frame').get_parameter_value().string_value
        self.standoff_distance_in_meters = self.get_parameter("stand_off_distance").get_parameter_value().double_value
        update_waypoint_topic = self.get_parameter('update_waypoint_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.get_state_service_name = self.get_parameter('get_state_service').get_parameter_value().string_value
        self.change_state_service_name = self.get_parameter('change_state_service').get_parameter_value().string_value
        self.get_waypoint_service_name = self.get_parameter('get_waypoint_service').get_parameter_value().string_value
        self.n_points = self.get_parameter('n_points').get_parameter_value().integer_value
        self.reacquisition_s_param = self.get_parameter('reacquisition_s_param').get_parameter_value().double_value
        self.update_rate = self.get_parameter('check_state_update_rate').get_parameter_value().integer_value

        #To remove surface reflections from FLS, this is the min depth, the vehicle must be at.
        # self.depth = -math.tan(math.radians(self.depth)) * self.standoff_distance_in_meters

        self.declare_parameter('search_mode_depth', Parameter.Type.DOUBLE)
        self.declare_parameter('search_mode_initial_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('search_mode_max_circles', Parameter.Type.INTEGER)
        self.declare_parameter('search_mode_timer', Parameter.Type.INTEGER)
        self.declare_parameter('follow_mode_timer',Parameter.Type.INTEGER)
        self.declare_parameter('exit_mode_distance',Parameter.Type.DOUBLE)
        self.declare_parameter('follow_mode_surge', Parameter.Type.DOUBLE)
        self.declare_parameter('reacquisition_surge', Parameter.Type.DOUBLE)
        self.declare_parameter('search_mode_surge', Parameter.Type.DOUBLE)
        self.declare_parameter('exit_mode_surge', Parameter.Type.DOUBLE)


        # Read parameters
        self.search_mode_depth = self.get_parameter('search_mode_depth').get_parameter_value().double_value
        self.search_mode_initial_radius = self.get_parameter('search_mode_initial_radius').get_parameter_value().double_value
        self.search_mode_max_circles = self.get_parameter('search_mode_max_circles').get_parameter_value().integer_value

        # Timers
        self.search_mode_timer_param = self.get_parameter('search_mode_timer').get_parameter_value().integer_value

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
        self.listener = tf2_ros.TransformListener(self.tf_buffer,self)

        self.node_name = self.get_name()

        self.poses = []

        self.count_concentric_circles = 0
        self.x, self.y = 0.0,0.0
        
        #Number of loops
        self.loop = 0

        self.plan_depth = False

        self.bool_search_mode = False

        self.bool_kill_state = False

        self.bool_exit_mode = False

        self.valid_best_point = True

        self.search_mode_complete = False
        self._reacquisition_active = False
        self.bool_initial_search = False

        self.mission_command = "EMPTY"

        self.base_to_odom_tf = None
        self.odom_to_base_tf = None

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

            if self.mission_command in ["START", "RESTART"]:
                self.bool_initial_search = True
                self.search_mode_complete = False
                self.count_concentric_circles = 0
                self.follow_flag = 0

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
        if self.base_to_odom_tf is None or self.odom_to_base_tf is None:
            return
        vx = self.base_to_odom_tf.transform.translation.x
        vy = self.base_to_odom_tf.transform.translation.y

        pt = PointStamped()
        pt.point.x = msg.x
        pt.point.y = msg.y
        pt_in_base = tf2_geometry_msgs.do_transform_point(pt, self.odom_to_base_tf)

        far_enough = math.hypot(vx - msg.x, vy - msg.y) > 5.0
        ahead = pt_in_base.point.x > 0

        if far_enough and ahead:
            self.x, self.y = msg.x, msg.y
            self.valid_best_point = True
        else:
            self.valid_best_point = False

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
            if not self.bool_kill_state:
                # n_wpt <= 3: 1 pre-existing helm wpt + 1 or 2 published wpts (best_point [+ farthest])
                if n_wpt <= 3:
                    # Vehicle is navigating away from iceberg after timeout
                    if self.bool_exit_mode:
                        msg.data = 2   # exit mode
                        self.pub_state.publish(msg)
                    else:
                        # Actively following iceberg: best_point + optional farthest sector point
                        msg.data = 0   # following
                        self.pub_state.publish(msg)
                # n_wpt > 3: 1 pre-existing + 8 arc/circle points (search or reacquisition arc)
                else:
                    # Vehicle is executing a concentric circle search pattern
                    if self.bool_search_mode:
                        msg.data = -1  # search
                        self.pub_state.publish(msg)
                    # Vehicle is executing a reacquisition arc after losing the iceberg
                    else:
                        if self.mission_command != "EMPTY":
                            msg.data = 1   # reacquisition
                            self.pub_state.publish(msg)
                        else:
                            # Mission not started yet
                            msg.data = -2  # idle
                            self.pub_state.publish(msg)
            else:
                msg.data = -3   # kill
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
        if self._reacquisition_active:
            return
        try:
            #Vx position and bearing in Odom frame.
            self.base_to_odom_tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame,
                                                                rclpy.time.Time())
            #Odom frame point in Vx frame
            self.odom_to_base_tf = self.tf_buffer.lookup_transform(self.base_frame, self.odom_frame,
                                                                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed in mission_core: {e}", throttle_duration_sec=5)
            return
        #Create Waypoint Message
        wpts = Waypoints()
        self.header = msg.header

        if self.bool_initial_search:
            self.bool_initial_search = False
            self.count_concentric_circles += 1
            self.search_mode(wpts)
            return

        # Valid path is n_points long.
        # Path is always being published.
        # If same path, then no new path is then published.
        if len(msg.poses) == self.n_points:

            self.farthest_sector_pose = None
            max_x = float('-inf')
            for pose_stamped in msg.poses:
                bl_pt = self.pose_in_base_link(pose_stamped)
                direction = math.degrees(math.atan2(bl_pt.point.y, bl_pt.point.x))
                if -10 < direction < 45 and bl_pt.point.x > max_x:
                    max_x = bl_pt.point.x
                    self.farthest_sector_pose = pose_stamped
                  
            valid_farthest_point = self.farthest_sector_pose is not None

            if self.state == "mapping":
                if self.valid_best_point:

                    #grab time of follow_mode initializing
                    if self.follow_flag == 0:
                        self.follow_mode_timer = time.time()
                        self.follow_flag = 1
                    
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
                        
                        if valid_farthest_point:
                            wpt = Waypoint()
                            wpt.header = msg.header
                            wpt.u = self.follow_mode_surge
                            best_point = Point()
                            best_point.x = self.farthest_sector_pose.pose.position.x
                            best_point.y = self.farthest_sector_pose.pose.position.y
                            best_point.z = self.depth
                            wpt.wpt = best_point
                            wpts.wpt.append(wpt)
                        self.pub_update.publish(wpts)

                        # self.poses = msg.poses            
                    #Chart a course away from the iceberg when timer runs out.
                    #Go to a point 90 degree port side of Vx
                    else:
                        if not self.bool_exit_mode:
                            self.get_logger().info(f"Exit sequence. Timer ran out at {self.follow_mode_timer_param}s")
                            self.exit_mode(wpts, info = f"Mission completed. Timeout of {self.follow_mode_timer_param}s.")
                    # else:
                    #     self.iceberg_reacquisition_mode(wpts)
                else:
                    return
            #Iceberg Reacquisition Mode is when
            #the vehicle reaches end of a valid path.
            elif self.state == "start":
                self.iceberg_reacquisition_mode(wpts)

            elif self.state == "kill":
                self.get_logger().warn("Helm switched to Kill", throttle_duration_sec=5)
                self.bool_kill_state = True

            elif self.state == "survey":
                if not self.bool_exit_mode:
                    self.get_logger().info(f"Mission interrupted. Moving to a safe point")
                    self.exit_mode(wpts, info = f"Mission interupted. Moving to a safe point")

        #Path is still published when no costmap. But the n_points is 1 (vx_x, vx_y)
        #We use that parameter to create a new bhvr mode.
        else:
            # rospy.loginfo("Searching Mode")
            if self.state == "start":
                if not self.search_mode_complete:
                    self.count_concentric_circles += 1
                    self.search_mode(wpts)
                else:
                    self.iceberg_reacquisition_mode(wpts)

            elif self.state == "mapping":
                #If timer runs out, then the node is killed.
                if(time.time() - self.search_mode_timer) > self.search_mode_timer_param: #sec
                    request = ChangeState.Request()
                    request.state = "kill"
                    request.caller = self.node_name
                    future = self.change_state_service_client.call_async(request)
                    future.add_done_callback(self.get_state_callback)

                    self.get_logger().warn("Search Mode Took too long --shutting down")
                    self.destroy_node()
                    rclpy.shutdown()


        # Find farthest path point in base_link that lies within the sector
    def pose_in_base_link(self, pose_stamped):
        pt = PointStamped()
        pt.header = self.header
        pt.point = pose_stamped.pose.position
        return tf2_geometry_msgs.do_transform_point(pt, self.odom_to_base_tf)   
    
         
    def exit_mode(self, wpts, info):
        """
        Function to navigate the vehicle 
        away from the iceberg when the timer runs out.
        """
        self.bool_exit_mode = True
        # Costmap frames only exists if path can be generated. If in Reacquisition mode,
        # then direct away from the iceberg
        if self.tf_buffer.can_transform(self.odom_frame, self.line_frame,
                                        rclpy.time.Time()) and self.tf_buffer.can_transform(self.line_frame,
                                                                                self.base_frame,
                                                                                rclpy.time.Time()):
            #Line_frame point in Odom Frame
            line_frame_to_odom_tf = self.tf_buffer.lookup_transform(self.odom_frame,
                                                                    self.line_frame,
                                                                rclpy.time.Time())

            vx_to_line_frame_tf = self.tf_buffer.lookup_transform(self.line_frame,
                                                                    self.base_frame,
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
        if not hasattr(self, '_exit_reset_timer') or self._exit_reset_timer.is_canceled():
            self._exit_reset_timer = self.create_timer(1.0, self._exit_mode_reset)

    def _exit_mode_reset(self):
        self._exit_reset_timer.cancel()
        self.mission_command = "EMPTY"
        self.count_concentric_circles = 0
        self.bool_search_mode = False
        self.follow_flag = 0
        self.bool_exit_mode = False
        self.search_mode_complete = False
        self.get_logger().info("Exit mode reset. Mission can be started again.")

    def search_mode(self, wpts):
        if self.count_concentric_circles > self.search_mode_max_circles:
            self.get_logger().warn(f"Search limit of {self.search_mode_max_circles} circles exceeded — shutting down")
            request = ChangeState.Request()
            request.state = "kill"
            request.caller = self.node_name
            future = self.change_state_service_client.call_async(request)
            future.add_done_callback(self.get_state_callback)
            self.destroy_node()
            return

        self.bool_search_mode = True
        self.search_mode_timer = time.time()

        search_mode_depth = self.search_mode_depth
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
        search_mode_points = self.draw_arc(number_of_points=8, 
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
        request.state = "mapping"
        request.caller = self.node_name
        future = self.change_state_service_client.call_async(request)
        future.add_done_callback(self.get_state_callback)
        self.pub_update.publish(wpts)
        self.search_mode_complete = True
        self.get_logger().info("Search Mode", throttle_duration_sec = 3)
        self.state = "mapping"

    def iceberg_reacquisition_mode(self, wpts):
        """
        Function to navigate the vehicle
        so as to reacquire acoustic contact
        """

        self._reacquisition_active = True

        request = ChangeState.Request()
        request.state = "mapping"
        request.caller = self.node_name
        future = self.change_state_service_client.call_async(request)
        future.add_done_callback(self.get_state_callback)

        point_of_obstacle = [self.reacquisition_s_param * self.standoff_distance_in_meters,
                             -self.standoff_distance_in_meters]
        corner_bhvr_points = self.draw_arc(number_of_points=8,
                                           start_angle=math.pi/2,
                                           end_angle=0,
                                           center=point_of_obstacle,
                                           radius=self.standoff_distance_in_meters)
        for pt in corner_bhvr_points:
            wpt = Waypoint()
            wpt.header = self.header
            wpt.u = self.reacquisition_surge
            msg = Point()
            msg.x = pt.point.x
            msg.y = pt.point.y
            msg.z = self.depth
            wpt.wpt = msg
            wpts.wpt.append(wpt)

        self.get_logger().info("Iceberg Reacquisition Mode", throttle_duration_sec=3)
        self.pub_update.publish(wpts)
        self._reacquisition_timer = self.create_timer(5.0, self._reacquisition_hold_done)

    def _reacquisition_hold_done(self):
        self._reacquisition_active = False
        self._reacquisition_timer.cancel()


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