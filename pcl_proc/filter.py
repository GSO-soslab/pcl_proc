#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from math import nan

class Filter(Node):
    def __init__(self):
        super().__init__('pcl_processing_node')

        # Declare parameters
        self.declare_parameter('stonefish', Parameter.Type.BOOL)
        if_stonefish = self.get_parameter('stonefish').get_parameter_value().bool_value

        if(if_stonefish):
            self.declare_parameter('sf_sub_topic', Parameter.Type.STRING)
            self.declare_parameter('sf_pub_topic', Parameter.Type.STRING)
            self.declare_parameter('sf_radial_filter_param', Parameter.Type.DOUBLE)
            self.declare_parameter('sf_std_dev_multiplier', Parameter.Type.DOUBLE)
            self.declare_parameter('sf_range_max', Parameter.Type.DOUBLE)
            self.declare_parameter('sf_number_of_bins', Parameter.Type.INTEGER)

            sub_topic = self.get_parameter('sf_sub_topic').get_parameter_value().string_value
            pub_topic = self.get_parameter('sf_pub_topic').get_parameter_value().string_value
            self.std_dev_multiplier = self.get_parameter('sf_std_dev_multiplier').get_parameter_value().double_value
            self.radial_filter = self.get_parameter('sf_radial_filter_param').get_parameter_value().double_value
            range_max = self.get_parameter('sf_range_max').get_parameter_value().double_value
            number_of_bins = self.get_parameter('sf_number_of_bins').get_parameter_value().integer_value


        else:
            self.declare_parameter('sub_topic')
            self.declare_parameter('pub_topic')
            self.declare_parameter('std_dev_multiplier')
            self.declare_parameter('radial_filter_param')
            self.declare_parameter('range_max')
            self.declare_parameter('number_of_bins')

            # Get parameters
            sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
            pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
            self.std_dev_multiplier = self.get_parameter('std_dev_multiplier').get_parameter_value().double_value
            self.radial_filter = self.get_parameter('radial_filter_param').get_parameter_value().double_value
            range_max = self.get_parameter('range_max').get_parameter_value().double_value
            number_of_bins = self.get_parameter('number_of_bins').get_parameter_value().integer_value

        self.bin_meter_coeff = number_of_bins / range_max

        # Create subscriber and publisher
        self.subscription = self.create_subscription(PointCloud2,sub_topic, self.pointcloud_callback, 10)

        self.publisher = self.create_publisher(PointCloud2,pub_topic,10)


    def pointcloud_callback(self, pointcloud_msg):
        pcl_msg = PointCloud2()
        pcl_msg.header.frame_id = pointcloud_msg.header.frame_id
        pcl_msg.header.stamp = pointcloud_msg.header.stamp
        
        pcl_msg.fields = pointcloud_msg.fields
        pcl_msg.height = pointcloud_msg.height
        pcl_msg.width = pointcloud_msg.width
        pcl_msg.point_step = pointcloud_msg.point_step
        pcl_msg.row_step = pointcloud_msg.row_step
        pcl_msg.is_dense = pointcloud_msg.is_dense


        mean, std_dev, median, intensities = self.get_intensities(pointcloud_msg=pointcloud_msg)
        # Populate filtered pointclouds.
        points = np.zeros((pcl_msg.width,len(pcl_msg.fields)),dtype=np.float32)
        pc_data = pc2.read_points(pointcloud_msg, skip_nans=True).tolist()
        for index,point in enumerate(pc_data):
            # Compare in bins
            if index >= self.radial_filter * self.bin_meter_coeff:
                ## Total bins = 1200. Set range = 20m. 1m = 60bins.
                ## Stonefish; bins = 100. Set range = 50m. 1m = 2 bins
                x, y, z, i = point[:4]
                #Filter
                if i > mean+self.std_dev_multiplier *std_dev and i>10:              
                    points[index][0] = x
                    points[index][1] = y
                    points[index][3] = i
                else:
                    points[index][0] = nan
                    points[index][1] = nan
                    points[index][3] = nan
            else:
                points[index][0] = nan
                points[index][1] = nan
                points[index][3] = nan
        
        pcl_msg.data = points.tobytes()
        self.publisher.publish(pcl_msg)

    def get_intensities(self,pointcloud_msg):
        """
        Returns the mean, std_dev and the echo intensity arrays.
        #To:DO Prpbably try out median filtering
        """
        intensities = []
        pc_data = pc2.read_points(pointcloud_msg, skip_nans=True).tolist()
        # print(len(pc_data[0]))_
        for index,point in enumerate(pc_data):
            #Only consider the intensities post the ringing filter.
            if index >= self.radial_filter * self.bin_meter_coeff:
                x, y, z, i = point[:4]
                intensities.append(i)
        intensities = np.array(intensities)
        mean = np.mean(intensities)
        std_dev = np.std(intensities)
        median = np.std(intensities)
        return mean, std_dev, median, intensities.tolist()

def main():
    rclpy.init()
    node = Filter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()