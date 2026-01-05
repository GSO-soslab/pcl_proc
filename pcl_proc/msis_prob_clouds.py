#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from scipy.spatial import cKDTree

class MSIS_Prob_Clouds(Node):

    def __init__(self):
        super().__init__('MSIS_Prob_Clouds')

        self.marker_sub = self.create_subscription(Marker,'/alpha_rise/msis/geometry',self.marker_cb,10)
        self.cloud_sub = self.create_subscription(PointCloud2,'/alpha_rise/msis/pointcloud', self.cloud_cb,10)
        self.pub = self.create_publisher(PointCloud2, '/alpha_rise/msis/pointcloud/fan', 10)
        self.receive_voxel_msg = False

        # Create angles
        self.cos_az, self.sin_az, self.cos_el, self.sin_el, self.az_grid = self.create_angles(2.0, 25.0, 1.0, 1.0)

        self.correspondence = False

    # -------- Marker callback ----------
    def marker_cb(self, msg: Marker):
        self.voxel_centroids = np.array([(p.x, p.y, p.z) for p in msg.points],dtype=np.float32)
        
        if not self.receive_voxel_msg:
            self.receive_voxel_msg = True

    # ---------- PointCloud callback ----------
    def cloud_cb(self, msg: PointCloud2):
        if self.receive_voxel_msg:
            gen = point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True)

            pointclouds = np.fromiter(
                gen,
                dtype=[('x', np.float32),
                    ('y', np.float32),
                    ('z', np.float32),
                    ('intensity', np.float32)])

            # Convert structured → normal array (N, 4)
            pointclouds = np.vstack([
                pointclouds['x'],
                pointclouds['y'],
                pointclouds['z'],
                pointclouds['intensity']
            ]).T

            # Range filtering
            dist_thresh = 5.0  # meters
            dist2_thresh = dist_thresh ** 2

            xyz = pointclouds[:, :3]
            dist2 = np.sum(xyz ** 2, axis=1)

            pointclouds = pointclouds[dist2 > dist2_thresh]

            # Populate SONAR fan
            pointclouds = self.populate_sonar_fan(pointclouds, self.cos_az, self.sin_az, self.cos_el, self.sin_el, self.az_grid)

            # Get correspondence
            if not self.correspondence:
                _, self.correspondence_index = self.create_voxel_corresponding_points(self.voxel_centroids, pointclouds[:,:3])
                self.correspondence = True
                
            pointclouds = pointclouds[self.correspondence_index, :]

            pointclouds = self.numpy_to_pointcloud2(pointclouds, msg)

            self.pub.publish(pointclouds)

    def create_voxel_corresponding_points(self, voxel_points:np.ndarray, geometry_points:np.ndarray):
        '''
        Finding the closest geometry_points corresponding to the voxel_points.
        
        :param geometry_points: List of geometry centroids (x,y,z)
        :param voxel_points: List of voxel centroids (x,y,z)

        :return list of geometry_points (x,y,z) that is closest correspondence with voxel_points. Same size as of voxel_points 
        '''
        voxel_points = np.asarray(voxel_points, dtype=np.float32)
        geometry_points = np.asarray(geometry_points, dtype=np.float32)

        # Build KD-tree on geometry_points.
        tree = cKDTree(geometry_points)

        # Query nearest neighbor for each voxel
        distance, indices = tree.query(voxel_points, k=1)

        return distance, indices
    
    def numpy_to_pointcloud2(self, cloud_np, msg):
        """
        Convert Nx4 NumPy array (x,y,z,intensity) to ROS2 PointCloud2
        """
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        cloud_msg = point_cloud2.create_cloud(
            header=msg.header,
            fields=fields,
            points=cloud_np
        )

        return cloud_msg
    

    def create_angles(self, az_bw_deg, el_bw_deg, az_step_deg, el_step_deg):
        # --- Generate angle grids ---
        az_angles = np.deg2rad(np.arange(-az_bw_deg/2, az_bw_deg/2 + az_step_deg, az_step_deg))  # (A,)
        el_angles = np.deg2rad(np.arange(-el_bw_deg/2, el_bw_deg/2 + el_step_deg, el_step_deg))  # (E,)
        
        az_grid, el_grid = np.meshgrid(az_angles, el_angles)  # (E,A)
        az_grid = az_grid.flatten()  # (B,)
        el_grid = el_grid.flatten()  # (B,)
        
        # --- Compute trigonometric values ---
        cos_az = np.cos(az_grid)[None, :]  # (1,B)
        sin_az = np.sin(az_grid)[None, :]  # (1,B)
        cos_el = np.cos(el_grid)[None, :]  # (1,B)
        sin_el = np.sin(el_grid)[None, :]  # (1,B)

        return cos_az, sin_az, cos_el, sin_el, az_grid

    def populate_sonar_fan(self, cloud_np, cos_az, sin_az, cos_el, sin_el, az_grid):
        """
        Duplicate rays across both azimuth and elevation beamwidths.
        
        cloud_np: (N, 4) [x, y, z, intensity]
        az_bw_deg: azimuth beamwidth in degrees
        el_bw_deg: elevation beamwidth in degrees
        az_step_deg: step size for azimuth in degrees
        el_step_deg: step size for elevation in degrees
        
        Returns: (M, 4) array of points duplicated across azimuth and elevation fan
        """
        
        # --- Compute ray lengths ---
        r = np.linalg.norm(cloud_np[:, :3], axis=1)[:, None]  # (N,1)
    
        
        # --- Normalize XY direction of original rays ---
        x_dir = cloud_np[:, 0:1] / r  # (N,1)
        y_dir = cloud_np[:, 1:2] / r  # (N,1)
        
        # --- Apply both azimuth and elevation fans ---
        x_fan = r * cos_el * (x_dir * cos_az - y_dir * sin_az)  # (N,B)
        y_fan = r * cos_el * (x_dir * sin_az + y_dir * cos_az)  # (N,B)
        z_fan = r * sin_el                                      # (N,B)
        
        # --- Repeat intensity ---
        intensity_fan = np.tile(cloud_np[:, 3:4], (1, len(az_grid)))  # (N,B)
        
        # --- Stack and reshape ---
        fan_points = np.stack([x_fan, y_fan, z_fan, intensity_fan], axis=-1)  # (N,B,4)
        fan_points = fan_points.reshape(-1, 4)  # (N*B, 4)
        
        return fan_points
def main():
    rclpy.init()
    node = MSIS_Prob_Clouds()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
