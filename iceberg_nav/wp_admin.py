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
from enum import Enum, auto


class Mode(Enum):
    IDLE          = auto()   # no mission active
    SEARCH        = auto()   # navigating search circle, waiting for iceberg contact
    FOLLOW        = auto()   # valid contact; publishing best_point/farthest waypoints
    REACQUISITION = auto()   # helm in "start" state, doing reacquisition arc
    EXIT          = auto()   # published exit waypoint
    KILL          = auto()   # helm killed


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

        # State machine
        self.mode = Mode.IDLE
        self.state = None   # helm state string

        # Mission context
        self.mission_command = "EMPTY"
        self.loop = 0
        self.count_concentric_circles = 0
        self.depth = 0.0

        # Timers (wallclock start times, None = not running)
        self.follow_mode_timer = None
        self.search_mode_timer = None
        self._search_circle_sent = False

        # Sensor data
        self.distance_to_obstacle = None
        self.x, self.y = 0.0, 0.0
        self.valid_best_point = False
        self.plan_depth = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.base_to_odom_tf = None
        self.odom_to_base_tf = None

        self.node_name = self.get_name()
        self.poses = []

        self.create_timer(self.update_rate, self.check_state)

        self.get_logger().info("Administrator launched. Use /alpha_rise/mission service to engage. START, RESTART or CONTINUE")

    # ------------------------------------------------------------------ #
    #  State machine                                                       #
    # ------------------------------------------------------------------ #

    def _transition(self, new_mode: Mode):
        self.get_logger().info(f"Mode: {self.mode.name} → {new_mode.name}")
        self.mode = new_mode

    # ------------------------------------------------------------------ #
    #  Service callbacks                                                   #
    # ------------------------------------------------------------------ #

    def mission_service_cb(self, request, response):
        """
        This service indicates whether to start, restart or continue the mapping.
        """
        if request.data in ["START", "RESTART", "CONTINUE"]:
            self.mission_command = request.data
            response.success = True
            response.message = "VALID CMD RECIEVED, EXECUTING MISSION"

            if self.mission_command in ["START", "RESTART"]:
                self._transition(Mode.SEARCH)
                self.count_concentric_circles = 0
                self.follow_mode_timer = None
                self._search_circle_sent = False
                self.valid_best_point = False
                for attr in ('_reacquisition_timer', '_reacquisition_wait_timer', '_exit_reset_timer'):
                    timer = getattr(self, attr, None)
                    if timer is not None:
                        timer.cancel()

            if self.mission_command == "CONTINUE":
                req = SetBool.Request()
                req.data = True
                future = self.depth_planner_service_client.call_async(req)
                future.add_done_callback(self.depth_planner_callback)
        else:
            response.success = False
            response.message = "INVALID CMD"
        return response

    def depth_planner_callback(self, future):
        future.result()

    def revisit_service_cb(self, request, response):
        """
        On this service call, either by loop detection or timeout, go to exit mode.
        """
        wpts = Waypoints()
        if request.data:
            response.success = True
            response.message = "Revisit triggered."
            self.loop += 1
            self.exit_mode(wpts, info=response.message)
        else:
            response.success = False
            response.message = "Revisit not triggered"
        print(response.message, flush=True)
        return response

    # ------------------------------------------------------------------ #
    #  Sensor callbacks                                                    #
    # ------------------------------------------------------------------ #

    def surge_cB(self, msg):
        self.plan_depth = (msg.data == 1)

    def point_cB(self, msg):
        """Best point callback."""
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
        """Distance to obstacle callback — triggers state publishing."""
        self.distance_to_obstacle = msg.data
        request = GetWaypoints.Request()
        request.count.data = 0
        future = self.get_waypoint_service_client.call_async(request)
        future.add_done_callback(self.get_n_waypoints)

    def get_n_waypoints(self, future):
        if future.done():
            mode_to_state = {
                Mode.KILL:          -3,
                Mode.EXIT:           2,
                Mode.SEARCH:        -1,
                Mode.REACQUISITION:  1,
                Mode.FOLLOW:         0,
                Mode.IDLE:          -2,
            }
            msg = Int16()
            msg.data = mode_to_state.get(self.mode, -2)
            self.pub_state.publish(msg)

    # ------------------------------------------------------------------ #
    #  Main path callback                                                  #
    # ------------------------------------------------------------------ #

    def path_cB(self, msg):
        """Path topic callback — drives the state machine."""
        if self.mode == Mode.IDLE:
            return
        if self.mission_command in ["START", "RESTART"]:
            self.depth = self.get_parameter('operating_depth').get_parameter_value().double_value
        elif self.mission_command == "CONTINUE":
            self.depth = (self.loop + 1) * self.get_parameter('operating_depth').get_parameter_value().double_value
        self.mission_core(msg)

    def mission_core(self, msg):
        # Modes where we let the helm execute its last waypoint undisturbed
        if self.mode in (Mode.IDLE, Mode.REACQUISITION, Mode.EXIT, Mode.KILL):
            return

        try:
            self.base_to_odom_tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
            self.odom_to_base_tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.odom_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed in mission_core: {e}", throttle_duration_sec=5)
            return

        wpts = Waypoints()
        self.header = msg.header

        # Helm-level interrupts (checked for all active modes)
        if self.state == "kill":
            self._transition(Mode.KILL)
            return
        if self.state == "survey":
            self.exit_mode(wpts, info="Interrupted by survey state.")
            return

        if self.mode == Mode.SEARCH:
            if not self._search_circle_sent:
                self.count_concentric_circles += 1
                self.search_mode(wpts)
                self._search_circle_sent = True
                return
            # Circle published — waiting for iceberg contact
            if self.valid_best_point and self.state == "mapping":
                self._transition(Mode.FOLLOW)
                self.follow_mode_timer = time.time()
            return

        if len(msg.poses) == self.n_points:
            self._handle_full_path(msg, wpts)
        else:
            self._handle_no_path(wpts)

    def _handle_full_path(self, msg, wpts):
        """Full costmap path available."""
        self.farthest_sector_pose = None
        max_x = float('-inf')
        for pose_stamped in msg.poses:
            bl_pt = self.pose_in_base_link(pose_stamped)
            direction = math.degrees(math.atan2(bl_pt.point.y, bl_pt.point.x))
            if -45 < direction < 45 and bl_pt.point.x > max_x:
                max_x = bl_pt.point.x
                self.farthest_sector_pose = pose_stamped
        valid_farthest_point = self.farthest_sector_pose is not None

        if self.state == "mapping":
            if self.valid_best_point:
                if self.follow_mode_timer is None:
                    self.follow_mode_timer = time.time()
                elapsed = time.time() - self.follow_mode_timer
                self.get_logger().info(
                    f"Following Mode with {round(self.follow_mode_timer_param - elapsed)}s remaining",
                    throttle_duration_sec=15)

                if elapsed < self.follow_mode_timer_param:
                    self._publish_follow_waypoints(msg, wpts, valid_farthest_point)
                else:
                    self.exit_mode(wpts, info=f"Follow timer expired at {self.follow_mode_timer_param}s.")

        elif self.state == "start":
            self.iceberg_reacquisition_mode(wpts)

    def _handle_no_path(self, wpts):
        """No costmap — short path."""
        if self.state == "start":
            self.iceberg_reacquisition_mode(wpts)

        elif self.state == "mapping":
            if self.search_mode_timer is not None and \
               (time.time() - self.search_mode_timer) > self.search_mode_timer_param:
                request = ChangeState.Request()
                request.state = "kill"
                request.caller = self.node_name
                future = self.change_state_service_client.call_async(request)
                future.add_done_callback(self.get_state_callback)
                self.get_logger().warn("Search mode took too long — shutting down")
                self.destroy_node()
                rclpy.shutdown()

    def _publish_follow_waypoints(self, msg, wpts, valid_farthest_point):
        wpt = Waypoint()
        wpt.header = msg.header
        wpt.u = 0.1 if self.plan_depth else self.follow_mode_surge
        best_point = Point()
        best_point.x = self.x
        best_point.y = self.y
        best_point.z = self.depth
        wpt.wpt = best_point
        wpts.wpt.append(wpt)

        if valid_farthest_point:
            wpt = Waypoint()
            wpt.header = msg.header
            wpt.u = self.follow_mode_surge
            farthest = Point()
            farthest.x = self.farthest_sector_pose.pose.position.x
            farthest.y = self.farthest_sector_pose.pose.position.y
            farthest.z = self.depth
            wpt.wpt = farthest
            wpts.wpt.append(wpt)

        self.pub_update.publish(wpts)

    # ------------------------------------------------------------------ #
    #  Helper                                                              #
    # ------------------------------------------------------------------ #

    def pose_in_base_link(self, pose_stamped):
        pt = PointStamped()
        pt.header = self.header
        pt.point = pose_stamped.pose.position
        return tf2_geometry_msgs.do_transform_point(pt, self.odom_to_base_tf)

    # ------------------------------------------------------------------ #
    #  Mode functions                                                      #
    # ------------------------------------------------------------------ #

    def exit_mode(self, wpts, info):
        """Navigate vehicle away from the iceberg."""
        self._transition(Mode.EXIT)

        if self.tf_buffer.can_transform(self.odom_frame, self.line_frame, rclpy.time.Time()) and \
           self.tf_buffer.can_transform(self.line_frame, self.base_frame, rclpy.time.Time()):
            line_frame_to_odom_tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.line_frame, rclpy.time.Time())
            vx_to_line_frame_tf = self.tf_buffer.lookup_transform(
                self.line_frame, self.base_frame, rclpy.time.Time())

            exit_point = PointStamped()
            exit_point.point.x = np.float64(0)
            if vx_to_line_frame_tf.transform.translation.y >= 0:
                exit_point.point.y = self.exit_mode_distance + self.standoff_distance_in_meters
            else:
                exit_point.point.y = -self.exit_mode_distance - self.standoff_distance_in_meters
            exit_point_odom = tf2_geometry_msgs.do_transform_point(exit_point, line_frame_to_odom_tf)
        else:
            exit_point = PointStamped()
            exit_point.point.x = np.float64(0)
            exit_point.point.y = self.exit_mode_distance
            exit_point_odom = tf2_geometry_msgs.do_transform_point(exit_point, self.base_to_odom_tf)

        exit_msg = Point()
        exit_msg.x = exit_point_odom.point.x
        exit_msg.y = exit_point_odom.point.y
        exit_msg.z = np.float64(0)

        wpt = Waypoint()
        wpt.header = self.header
        wpt.wpt = exit_msg
        wpt.u = self.exit_mode_surge
        wpts.wpt.append(wpt)
        self.pub_update.publish(wpts)

        self.get_logger().info(info)
        if not hasattr(self, '_exit_reset_timer') or self._exit_reset_timer.is_canceled():
            self._exit_reset_timer = self.create_timer(1.0, self._exit_mode_reset)

    def _exit_mode_reset(self):
        if self.state != "start":
            return
        self._exit_reset_timer.cancel()
        self._transition(Mode.IDLE)
        self.mission_command = "EMPTY"
        self.count_concentric_circles = 0
        self.follow_mode_timer = None
        self.get_logger().info("Exit mode reset. Mission can be started again.")

    def search_mode(self, wpts):
        """Publish a concentric search circle. Caller owns mode transition."""
        if self.count_concentric_circles > self.search_mode_max_circles:
            self.get_logger().warn(
                f"Search limit of {self.search_mode_max_circles} circles exceeded — shutting down")
            request = ChangeState.Request()
            request.state = "kill"
            request.caller = self.node_name
            future = self.change_state_service_client.call_async(request)
            future.add_done_callback(self.get_state_callback)
            self.destroy_node()
            return

        self.search_mode_timer = time.time()

        if self.count_concentric_circles == 1:
            self.search_mode_center = PointStamped()
            self.search_mode_center.point.x = self.base_to_odom_tf.transform.translation.x
            self.search_mode_center.point.y = self.base_to_odom_tf.transform.translation.y

        center_in_vx_frame = tf2_geometry_msgs.do_transform_point(
            self.search_mode_center, self.odom_to_base_tf)

        search_mode_radius = self.search_mode_initial_radius * self.count_concentric_circles
        search_mode_points = self.draw_arc(
            number_of_points=8,
            start_angle=0,
            end_angle=2 * math.pi,
            center=[center_in_vx_frame.point.x, center_in_vx_frame.point.y],
            radius=search_mode_radius)

        for pt in search_mode_points:
            wpt = Waypoint()
            wpt.header = self.header
            msg = Point()
            msg.x = pt.point.x
            msg.y = pt.point.y
            msg.z = self.search_mode_depth
            wpt.wpt = msg
            wpt.u = self.search_mode_surge
            wpts.wpt.append(wpt)

        request = ChangeState.Request()
        request.state = "mapping"
        request.caller = self.node_name
        future = self.change_state_service_client.call_async(request)
        future.add_done_callback(self.get_state_callback)
        self.pub_update.publish(wpts)
        self.get_logger().info("Search Mode", throttle_duration_sec=3)
        self.state = "mapping"

    def iceberg_reacquisition_mode(self, wpts):
        """Publish a reacquisition arc and hold for 5 s."""
        self._transition(Mode.REACQUISITION)

        if self.state != "mapping":
            request = ChangeState.Request()
            request.state = "mapping"
            request.caller = self.node_name
            future = self.change_state_service_client.call_async(request)
            future.add_done_callback(self.get_state_callback)

        point_of_obstacle = [self.reacquisition_s_param * self.standoff_distance_in_meters,
                              -self.standoff_distance_in_meters]
        corner_bhvr_points = self.draw_arc(
            number_of_points=8,
            start_angle=math.pi / 2,
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
        self._reacquisition_timer.cancel()
        if self.state == "kill":
            self._transition(Mode.KILL)
        elif self.state == "survey":
            self.exit_mode(Waypoints(), info="Interrupted by survey state.")
        elif self.valid_best_point:
            self._transition(Mode.FOLLOW)
        else:
            self._reacquisition_wait_timer = self.create_timer(1.0, self._reacquisition_wait_check)

    def _reacquisition_wait_check(self):
        """Poll at 1 Hz after hold expires; re-trigger arc only when helm is back in 'start'."""
        if self.state == "kill":
            self._reacquisition_wait_timer.cancel()
            self._transition(Mode.KILL)
        elif self.state == "survey":
            self._reacquisition_wait_timer.cancel()
            self.exit_mode(Waypoints(), info="Interrupted by survey state.")
        elif self.valid_best_point:
            self._reacquisition_wait_timer.cancel()
            self._transition(Mode.FOLLOW)
        elif self.state == "start":
            self._reacquisition_wait_timer.cancel()
            self.iceberg_reacquisition_mode(Waypoints())

    # ------------------------------------------------------------------ #
    #  Helm state polling                                                  #
    # ------------------------------------------------------------------ #

    def check_state(self):
        request = GetState.Request()
        future = self.get_state_service_client.call_async(request)
        future.add_done_callback(self.get_state_callback)

    def get_state_callback(self, future):
        response = future.result()
        self.state = response.state.name

    # ------------------------------------------------------------------ #
    #  Geometry helpers                                                    #
    # ------------------------------------------------------------------ #

    def draw_arc(self, number_of_points, start_angle, end_angle, center, radius):
        """Create an arc in vehicle frame and transform to odom."""
        angles = np.linspace(start_angle, end_angle, number_of_points)
        corner_bhvr_points = [(center[0] + radius * math.cos(a),
                               center[1] + radius * math.sin(a))
                              for a in angles]
        points_in_odom_frame = []
        for p in corner_bhvr_points:
            point_msg = PointStamped()
            point_msg.point.x = p[0]
            point_msg.point.y = p[1]
            points_in_odom_frame.append(
                tf2_geometry_msgs.do_transform_point(point_msg, self.base_to_odom_tf))
        return points_in_odom_frame


def main():
    rclpy.init()
    node = Wp_Admin()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
