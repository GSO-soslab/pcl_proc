#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial import cKDTree
import tf2_ros
from tf2_ros import TransformException
import tf2_sensor_msgs.tf2_sensor_msgs

class MsisProbClouds(Node):

    def __init__(self):
        super().__init__('msis_prob_clouds')

        self.receive_voxel_msg = False

        # Parameters
        self.declare_parameter('marker_topic', Parameter.Type.STRING)
        self.declare_parameter('cloud_sub_topic', Parameter.Type.STRING)
        self.declare_parameter('fan_pub_topic', Parameter.Type.STRING)
        self.declare_parameter('vertical_fov_deg', Parameter.Type.DOUBLE)
        self.declare_parameter('resolution', Parameter.Type.DOUBLE)
        self.declare_parameter('min_range', Parameter.Type.DOUBLE)
        self.declare_parameter('noise_floor', Parameter.Type.DOUBLE)
        self.declare_parameter('z_max', Parameter.Type.DOUBLE)
        self.declare_parameter('world_frame_id', Parameter.Type.STRING)
        self.sensor_frame_id = ''
        self.declare_parameter('sound_speed', Parameter.Type.DOUBLE)
        self.declare_parameter('frequency', Parameter.Type.DOUBLE)
        self.declare_parameter('aperture_size', Parameter.Type.DOUBLE)

        marker_topic = self.get_parameter('marker_topic').value
        cloud_sub_topic = self.get_parameter('cloud_sub_topic').value
        fan_pub_topic = self.get_parameter('fan_pub_topic').value

        self.marker_sub = self.create_subscription(Marker, marker_topic, self.marker_cb, 10)
        self.cloud_sub = self.create_subscription(PointCloud2, cloud_sub_topic, self.cloud_cb, 10)
        self.pub = self.create_publisher(PointCloud2, fan_pub_topic, 10)

        v_fov_deg = self.get_parameter('vertical_fov_deg').value
        resolution = self.get_parameter('resolution').value
        self.min_range = self.get_parameter('min_range').value
        self.noise_floor = self.get_parameter('noise_floor').value
        self.z_max = self.get_parameter('z_max').value
        self.world_frame_id = self.get_parameter('world_frame_id').value

        # Create elevation angle arrays
        el_angles_deg = np.arange(-v_fov_deg / 2, v_fov_deg / 2 + resolution, resolution)
        el_angles = np.deg2rad(el_angles_deg)
        self.cos_el = np.cos(el_angles)[None, :]  # (1, E)
        self.sin_el = np.sin(el_angles)[None, :]  # (1, E)
        # Sinc beam directivity: DI = sin(k*h/2*sin(θ)) / (k*h/2*sin(θ))
        sound_speed = self.get_parameter('sound_speed').value
        frequency   = self.get_parameter('frequency').value
        aperture    = self.get_parameter('aperture_size').value
        wavelength  = sound_speed / frequency
        k           = 2 * np.pi / wavelength
        temp        = (k * aperture / 2) * np.sin(el_angles)
        di          = np.ones_like(temp, dtype=np.float32)
        nz          = np.abs(temp) > 1e-10
        di[nz]      = np.sin(temp[nz]) / temp[nz]
        self.el_prob = np.array([round(float(v), 2) for v in di], dtype=np.float32)[None, :]  # (1, E)

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
        self.sensor_frame_id = msg.header.frame_id
        self.voxel_centroids = np.array([(p.x, p.y, p.z) for p in msg.points], dtype=np.float32)
        self.voxel_tree = cKDTree(self.voxel_centroids)
        self.voxel_resolution = msg.scale.x
        if not self.receive_voxel_msg:
            self.receive_voxel_msg = True

    # ---------- PointCloud callback ----------
    def cloud_cb(self, msg: PointCloud2):
        if not self.receive_voxel_msg:
            return

        pointclouds = self.parse_buffer(msg)
        pointclouds = self.range_filter(pointclouds)
        pointclouds = self.voxel_max(pointclouds)
        pointclouds = self.convert_to_probabilities(pointclouds)
        fan_points  = self.populate_sonar_fan(pointclouds, self.cos_el, self.sin_el)
        fan_points  = self.snap_fan_to_voxels(fan_points)
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

    def range_filter(self, pointclouds):
        """Range filter"""
        mask = (np.isfinite(pointclouds).all(axis=1) &
                ((pointclouds[:, 0]**2 + pointclouds[:, 1]**2 + pointclouds[:, 2]**2) > self.min_range**2))
        profile = np.zeros(len(pointclouds), dtype=np.float32)
        profile[mask] = pointclouds[mask, 3]
        return pointclouds[mask]

    def depth_filter(self, pointcloud_msg):
        """Transform to world frame, filter points above z_max, transform back to sensor frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame_id,
                self.sensor_frame_id,
                rclpy.time.Time()
            )

            pointcloud_msg = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)

            pts = np.frombuffer(pointcloud_msg.data, dtype=self._parse_dtype)
            points = np.column_stack([pts['x'], pts['y'], pts['z'], pts['intensity']])

            points = points[points[:, 2] <= self.z_max]
            pointcloud_msg.data = points.tobytes()
            pointcloud_msg.width = len(points)
            pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.sensor_frame_id,
                    self.world_frame_id,
                    rclpy.time.Time()
                )
                return tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)

            except TransformException as e:
                self.get_logger().warn(f'depth_filter: inverse TF lookup failed: {e}')
                return self.empty_cloud(pointcloud_msg.header)

        except TransformException as e:
            self.get_logger().warn(f'depth_filter: TF lookup failed: {e}')
            return self.empty_cloud(pointcloud_msg.header)

    def empty_cloud(self, header):
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
        
        # Find point with max intensity inside the voxel.
        best = np.array([
            first[g] + np.argmax(sorted_pts[first[g]:first[g] + counts[g], 3])
            for g in range(len(first))
        ], dtype=np.intp)
        # Map intensity to voxel centroid.
        result = sorted_pts[best].copy()
        result[:, :3] = self.voxel_centroids[sorted_vox[best]]
        return result

    def snap_fan_to_voxels(self, fan_points):
        """Snap each fan arc point to its nearest voxel centroid; discard out-of-bounds; keep max intensity per voxel."""
        if len(fan_points) == 0:
            return fan_points
        _, voxel_idx = self.voxel_tree.query(fan_points[:, :3], k=1, workers=-1)
        half = self.voxel_resolution / 2.0
        diff = np.abs(fan_points[:, :3] - self.voxel_centroids[voxel_idx])
        in_voxel = np.all(diff <= half, axis=1)
        fan_points = fan_points[in_voxel]
        voxel_idx = voxel_idx[in_voxel]
        if len(fan_points) == 0:
            return fan_points
        order = np.argsort(voxel_idx)
        sorted_vox = voxel_idx[order]
        sorted_pts = fan_points[order]
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
        """Convert intensity to occupancy probability: <noise floor→0.2, ≥noise floor→0.9."""
        prob = np.where(pointclouds[:, 3] >= self.noise_floor, 0.9, 0.2)
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
        # Joint probability
        fan_points[:, 3] = (intensity[:, None] * self.el_prob).reshape(-1)

        return fan_points

def main():
    rclpy.init()
    node = MsisProbClouds()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
