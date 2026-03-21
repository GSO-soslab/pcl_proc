#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from scipy.spatial import cKDTree
import tf2_ros
from tf2_ros import TransformException
import tf2_sensor_msgs.tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2

class MsisProbClouds(Node):

    def __init__(self):
        super().__init__('msis_prob_clouds')

        self.receive_voxel_msg = False

        # Parameters
        self.declare_parameter('marker_topic', Parameter.Type.STRING)
        self.declare_parameter('cloud_sub_topic', Parameter.Type.STRING)
        self.declare_parameter('fan_pub_topic', Parameter.Type.STRING)
        self.declare_parameter('range_filter_pub_topic', Parameter.Type.STRING)
        self.declare_parameter('vertical_fov_deg', Parameter.Type.DOUBLE)
        self.declare_parameter('resolution', Parameter.Type.DOUBLE)
        self.declare_parameter('min_range', Parameter.Type.DOUBLE)
        self.declare_parameter('intensity_threshold', Parameter.Type.DOUBLE)
        self.declare_parameter('z_max', Parameter.Type.DOUBLE)

        marker_topic = self.get_parameter('marker_topic').value
        cloud_sub_topic = self.get_parameter('cloud_sub_topic').value
        fan_pub_topic = self.get_parameter('fan_pub_topic').value
        range_filter_pub_topic = self.get_parameter('range_filter_pub_topic').value

        self.marker_sub = self.create_subscription(Marker, marker_topic, self.marker_cb, 10)
        self.cloud_sub = self.create_subscription(PointCloud2, cloud_sub_topic, self.cloud_cb, 10)
        self.pub = self.create_publisher(PointCloud2, fan_pub_topic, 10)
        self.pub_range_filter_profile = self.create_publisher(Float32MultiArray, range_filter_pub_topic, 10)

        v_fov_deg = self.get_parameter('vertical_fov_deg').value
        resolution = self.get_parameter('resolution').value
        self.min_range_ = self.get_parameter('min_range').value
        self.intensity_threshold = self.get_parameter('intensity_threshold').value
        self.z_max_ = self.get_parameter('z_max').value

        # Create elevation angle arrays
        el_angles = np.deg2rad(np.arange(-v_fov_deg / 2, v_fov_deg / 2 + resolution, resolution))
        self.cos_el = np.cos(el_angles)[None, :]  # (1, E)
        self.sin_el = np.sin(el_angles)[None, :]  # (1, E)

        el_angles_deg = np.arange(-v_fov_deg / 2, v_fov_deg / 2 + resolution, resolution)
        #1.0 to 0.6
        self.el_prob = (1.0 - 0.4 * np.abs(el_angles_deg) / (v_fov_deg / 2)).astype(np.float32)  # (E,)

        # Static voxel KDTree built once in marker_cb
        self.voxel_tree = None

        # Prebuilt PointCloud2 fields
        self._fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Structured dtype for zero-copy buffer parse, resolved on first message
        self._parse_dtype = None

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    # -------- Marker callback ----------
    def marker_cb(self, msg: Marker):
        self.voxel_centroids = np.array([(p.x, p.y, p.z) for p in msg.points], dtype=np.float32)
        self.voxel_tree = cKDTree(self.voxel_centroids)
        self.receive_voxel_msg = True

    # ---------- PointCloud callback ----------
    def cloud_cb(self, msg: PointCloud2):
        if not self.receive_voxel_msg:
            return

        pointclouds = self.parse_buffer(msg)
        N = len(pointclouds)
        pointclouds, _ = self.range_filter(pointclouds, N, min_range=self.min_range_)
        pointclouds = self.voxel_max(pointclouds)
        pointclouds = self.convert_to_probabilities(pointclouds)
        fan_points  = self.populate_sonar_fan(pointclouds, self.cos_el, self.sin_el)
        cloud_msg   = self.build_cloud_msg(msg.header, fan_points)
        cloud_msg   = self.depth_filter(cloud_msg)
        self.pub.publish(cloud_msg)

    # ---------- Hot-path steps ----------
    def parse_buffer(self, msg: PointCloud2):
        """Zero-copy parse of the raw ROS buffer → (N, 4) float32 array."""
        if self._parse_dtype is None:
            offsets = [next(f.offset for f in msg.fields if f.name == n)
                       for n in ('x', 'y', 'z', 'intensity')]
            self._parse_dtype = np.dtype({
                'names': ['x', 'y', 'z', 'intensity'],
                'formats': [np.float32] * 4,
                'offsets': offsets,
                'itemsize': msg.point_step,
            })
        pts = np.frombuffer(msg.data, dtype=self._parse_dtype)
        return np.column_stack([pts['x'], pts['y'], pts['z'], pts['intensity']])

    def range_filter(self, pointclouds, N, min_range=5.0):
        """Range filter + publish aligned N-length intensity profile. Returns (filtered, mask)."""
        mask = (np.isfinite(pointclouds).all(axis=1) &
                ((pointclouds[:, 0]**2 + pointclouds[:, 1]**2 + pointclouds[:, 2]**2) > min_range**2))
        profile = np.zeros(N, dtype=np.float32)
        profile[mask] = pointclouds[mask, 3]
        msg = Float32MultiArray()
        msg.data = profile.tolist()
        self.pub_range_filter_profile.publish(msg)
        return pointclouds[mask], mask

    def depth_filter(self, pointcloud_msg):
        """Transform to world frame, filter points above z_max, transform back to sensor frame."""
        sensor_frame = pointcloud_msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                'alpha_rise/world',
                sensor_frame,
                rclpy.time.Time()
            )

            pointcloud_msg = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)

            points_struct = pc2.read_points(
                pointcloud_msg,
                field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=False
            )

            points = np.column_stack((
                points_struct['x'],
                points_struct['y'],
                points_struct['z'],
                points_struct['intensity']
            )).astype(np.float32)

            points = points[points[:, 2] <= self.z_max_]
            pointcloud_msg = pc2.create_cloud(pointcloud_msg.header, self._fields, [tuple(p) for p in points])

            try:
                transform = self.tf_buffer.lookup_transform(
                    sensor_frame,
                    'alpha_rise/world',
                    rclpy.time.Time()
                )
                return tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)

            except TransformException as e:
                self.get_logger().warn(f'depth_filter: inverse TF lookup failed: {e}')
                return self._empty_cloud(pointcloud_msg.header)

        except TransformException as e:
            self.get_logger().warn(f'depth_filter: TF lookup failed: {e}')
            return self._empty_cloud(pointcloud_msg.header)

    def _empty_cloud(self, header):
        empty = PointCloud2()
        empty.header = header
        empty.height = 1
        empty.width = 0
        empty.fields = self._fields
        empty.is_bigendian = False
        empty.point_step = 16
        empty.row_step = 0
        empty.is_dense = True
        empty.data = b''
        return empty

    def voxel_max(self, pointclouds):
        """Assign each point to its nearest voxel centroid; keep max intensity per voxel."""
        if len(pointclouds) == 0:
            return pointclouds
        _, voxel_idx = self.voxel_tree.query(pointclouds[:, :3], k=1, workers=-1)
        order = np.argsort(voxel_idx)
        sorted_vox = voxel_idx[order]
        sorted_pts = pointclouds[order]
        _, first, counts = np.unique(sorted_vox, return_index=True, return_counts=True)
        best = np.array([
            first[g] + np.argmax(sorted_pts[first[g]:first[g] + counts[g], 3])
            for g in range(len(first))
        ], dtype=np.intp)
        result = sorted_pts[best].copy()
        result[:, :3] = self.voxel_centroids[sorted_vox[best]]
        return result

    def build_cloud_msg(self, header, result):
        """Pack a (V, 4) float32 array into a PointCloud2 message."""
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(result)
        cloud_msg.fields = self._fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = 16 * len(result)
        cloud_msg.is_dense = True
        cloud_msg.data = result.tobytes()
        return cloud_msg

    def convert_to_probabilities(self, pointclouds):
        """Convert intensity to occupancy probability: <20→0.2, ≥20→0.9."""
        prob = np.where(pointclouds[:, 3] >= self.intensity_threshold, 0.9, 0.2)
        pointclouds[:, 3] = prob
        return pointclouds

    def populate_sonar_fan(self, cloud_np, cos_el, sin_el):
        """
        cloud_np: (N, 4) [x, y, z, intensity]
        cos_el, sin_el: (1, E) elevation trig values
        Returns: (N*E, 4)
        """
        xyz = cloud_np[:, :3]
        intensity = cloud_np[:, 3]

        r = np.linalg.norm(xyz, axis=1, keepdims=True)
        r = np.maximum(r, 1e-6)

        inv_r = 1.0 / r
        x_dir = xyz[:, 0:1] * inv_r
        y_dir = xyz[:, 1:2] * inv_r

        rc = r * cos_el  # (N,E)

        fan_points = np.empty((xyz.shape[0] * cos_el.shape[1], 4), dtype=cloud_np.dtype)
        fan_points[:, 0] = (rc * x_dir).reshape(-1)
        fan_points[:, 1] = (rc * y_dir).reshape(-1)
        fan_points[:, 2] = (r * sin_el).reshape(-1)
        #Joint probability
        fan_points[:, 3] = (intensity[:, None] * self.el_prob[None, :]).reshape(-1)

        return fan_points

def main():
    rclpy.init()
    node = MsisProbClouds()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
