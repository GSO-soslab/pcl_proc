#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial import cKDTree

class MSIS_Prob_Clouds(Node):

    def __init__(self):
        super().__init__('MSIS_Prob_Clouds')

        self.marker_sub = self.create_subscription(Marker,'/alpha_rise/msis/geometry',self.marker_cb,10)
        self.cloud_sub = self.create_subscription(PointCloud2,'/alpha_rise/msis/pointcloud', self.cloud_cb,10)
        self.pub = self.create_publisher(PointCloud2, '/alpha_rise/msis/pointcloud/fan', 10)
        self.receive_voxel_msg = False

        # Create angles
        self.cos_az, self.sin_az, self.cos_el, self.sin_el, self.az_grid = self.create_angles(25.0, 1.0)

        # Static voxel KDTree built once in marker_cb
        self.voxel_tree = None

        # Prebuilt PointCloud2 fields — eliminates repeated allocation in hot path (Fix 4)
        self._fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Structured dtype for zero-copy buffer parse, resolved on first message (Fix 2)
        self._parse_dtype = None


    # -------- Marker callback ----------
    def marker_cb(self, msg: Marker):
        if not self.receive_voxel_msg:
            self.voxel_centroids = np.array([(p.x, p.y, p.z) for p in msg.points],dtype=np.float32)
            self.voxel_tree = cKDTree(self.voxel_centroids)   # built once, never again (Fix 1)
            self.receive_voxel_msg = True
            self.destroy_subscription(self.marker_sub)
            self.marker_sub = None

    # ---------- PointCloud callback ----------
    def cloud_cb(self, msg: PointCloud2):
        if not self.receive_voxel_msg:
            return

        pointclouds = self.parse_buffer(msg)
        pointclouds = self.filter_points_by_range(pointclouds)
        # pointclouds = self.apply_cfar(pointclouds, num_train=8, num_guard=2, false_alarm_rate=1e-3, boost=2.0)
        pointclouds = self.convert_to_probabilities(pointclouds, intensity_lower=20, intensity_upper=50)
        fan_points  = self.populate_sonar_fan(pointclouds, self.cos_el, self.sin_el)
        result      = self.find_correspondance_with_voxels(fan_points)
        self.pub.publish(self.build_cloud_msg(msg.header, result))

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

    def filter_points_by_range(self, pointclouds):
        """Single-pass NaN + range filter (r > 5 m)."""
        mask_finite = np.isfinite(pointclouds).all(axis=1)
        mask_range  = (pointclouds[:, 0]**2 + pointclouds[:, 1]**2 + pointclouds[:, 2]**2) > 25.0
        return pointclouds[mask_finite & mask_range]

    def find_correspondance_with_voxels(self, fan_points):
        """Query voxel tree; keep the closest fan point per voxel."""
        dist, voxel_idx = self.voxel_tree.query(fan_points[:, :3], k=1, workers=-1)
        order = np.argsort(dist)
        _, first = np.unique(voxel_idx[order], return_index=True)
        return fan_points[order[first]]

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
    
    def apply_range_filter(self, pointclouds, dist_thresh):
        dist2 = np.sum(pointclouds[:, :3] ** 2, axis=1)
        return pointclouds[dist2 > dist_thresh ** 2]

    def apply_cfar(self, pointclouds, num_train=8, num_guard=2, false_alarm_rate=1e-3, boost=2.0):
        """
        CA-CFAR (Cell Averaging CFAR) along the range axis.

        For each range-sorted cell, the noise level is estimated from num_train
        training cells on each side, separated from the test cell by num_guard
        guard cells.  Cells whose intensity exceeds the adaptive threshold are
        considered targets and their intensity is multiplied by `boost`.

        :param num_train:        training cells per side
        :param num_guard:        guard cells per side
        :param false_alarm_rate: desired PFA; sets the CFAR scaling factor alpha
        :param boost:            intensity multiplier applied to detected targets
        """
        if len(pointclouds) == 0:
            return pointclouds

        # Sort by range so cells are ordered along the scan
        ranges = np.linalg.norm(pointclouds[:, :3], axis=1)
        order = np.argsort(ranges)
        pts = pointclouds[order].copy()
        intensity = pts[:, 3]
        N = len(intensity)

        # CFAR threshold factor: alpha = N_train * (PFA^(-1/N_train) - 1)
        n_total_train = 2 * num_train
        alpha = n_total_train * (false_alarm_rate ** (-1.0 / n_total_train) - 1.0)

        half_win = num_train + num_guard
        target_mask = np.zeros(N, dtype=bool)

        for i in range(N):
            lo_guard = max(0, i - num_guard)
            hi_guard = min(N, i + num_guard + 1)
            lo_train = max(0, i - half_win)
            hi_train = min(N, i + half_win + 1)

            # Training cells = window minus guard band minus test cell
            train_idx = np.concatenate([
                np.arange(lo_train, lo_guard),
                np.arange(hi_guard, hi_train)
            ])

            if len(train_idx) == 0:
                continue

            noise_level = np.mean(intensity[train_idx])
            if noise_level > 0 and intensity[i] > alpha * noise_level:
                target_mask[i] = True

        pts[target_mask, 3] *= boost

        return pts[target_mask]

    def apply_median_filter(self, pointclouds, k):
        """
        Only keeps intensities > median + k*std_dev
        
        :param self: Description
        :param pointclouds: Description
        :param k: Description
        """
        intensity = pointclouds[:, 3]
        above_min = intensity > 10
        threshold = np.median(intensity) + k * np.std(intensity)
        above_threshold = intensity>threshold
        mask = above_min & above_threshold
        return pointclouds[mask]

    def convert_to_probabilities(self, pointclouds, intensity_lower=100, intensity_upper=200):
        """
        Convert intensity values to occupancy probabilities for mapping.

        Args:
            pointclouds: (N,4) [x, y, z, intensity]
            intensity_lower: intensity threshold mapping to 0.5; below this → 0.1
            intensity_upper: intensity threshold mapping to 0.9; above this → 0.9

        Returns:
            (N,4) array with intensity replaced by probability
        """

        intensity = pointclouds[:, 3]

        # Linear mapping above lower bound
        prob = 0.5 + 0.4 * (intensity - intensity_lower) / (intensity_upper - intensity_lower)

        # Clamp
        prob[intensity < intensity_lower] = 0.1
        prob[intensity >= intensity_upper] = 0.9
        prob = np.clip(prob, 0.1, 0.9)

        # Replace intensity
        pointclouds[:, 3] = prob
        return pointclouds


    def create_angles(self, el_bw_deg, el_step_deg):
        # --- Generate angle grids ---
        az_angles = np.array([0.0])  # no azimuth spread
        el_angles = np.deg2rad(np.arange(-el_bw_deg/2, el_bw_deg/2 + el_step_deg, el_step_deg))  # (E,)

        az_grid, el_grid = np.meshgrid(az_angles, el_angles)  # (E,1)
        az_grid = az_grid.flatten()  # (E,)
        el_grid = el_grid.flatten()  # (E,)
        
        # --- Compute trigonometric values ---
        cos_az = np.cos(az_grid)[None, :]  # (1,B)
        sin_az = np.sin(az_grid)[None, :]  # (1,B)
        cos_el = np.cos(el_grid)[None, :]  # (1,B)
        sin_el = np.sin(el_grid)[None, :]  # (1,B)

        return cos_az, sin_az, cos_el, sin_el, az_grid

    def populate_sonar_fan(self, cloud_np, cos_el, sin_el):
        """
        cloud_np: (N, 4) [x, y, z, intensity]
        cos_el, sin_el: (1, E) elevation trig values
        Returns: (N*E, 4)
        """

        # --- Extract ---
        xyz = cloud_np[:, :3]
        intensity = cloud_np[:, 3]

        N = xyz.shape[0]
        E = cos_el.shape[1]

        # --- Range ---
        r = np.linalg.norm(xyz, axis=1, keepdims=True)
        r = np.maximum(r, 1e-6)  # safety

        inv_r = 1.0 / r
        x_dir = xyz[:, 0:1] * inv_r
        y_dir = xyz[:, 1:2] * inv_r

        # --- Fan computation (azimuth=0, so cos_az=1, sin_az=0) ---
        rc = r * cos_el  # (N,E)

        x_fan = rc * x_dir
        y_fan = rc * y_dir
        z_fan = r * sin_el

        # --- Output buffer ---
        fan_points = np.empty((N * E, 4), dtype=cloud_np.dtype)

        fan_points[:, 0] = x_fan.reshape(-1)
        fan_points[:, 1] = y_fan.reshape(-1)
        fan_points[:, 2] = z_fan.reshape(-1)
        fan_points[:, 3] = np.repeat(intensity, E)

        return fan_points

def main():
    rclpy.init()
    node = MSIS_Prob_Clouds()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
