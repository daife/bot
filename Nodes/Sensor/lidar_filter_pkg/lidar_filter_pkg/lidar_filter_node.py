#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        
        # Create a subscriber to the MS200/scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            'MS200/scan',
            self.scan_callback,
            10)
        
        # Create a publisher for filtered scan data
        self.publisher = self.create_publisher(
            LaserScan,
            'scan',
            10)
        
        # Filter parameters (in degrees)
        self.min_angle = -45.0  # -45 degrees
        self.max_angle = 135.0  # +135 degrees
        
        self.get_logger().info('Lidar filter node started')
        self.get_logger().info(f'Filtering angles from {self.min_angle}° to {self.max_angle}°')

    def scan_callback(self, msg):
        # Create a copy of the incoming LaserScan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        
        # Convert filter angles from degrees to radians
        min_angle_rad = math.radians(self.min_angle)
        max_angle_rad = math.radians(self.max_angle)
        
        # Filter the ranges and intensities
        ranges = []
        intensities = []
        
        for i in range(len(msg.ranges)):
            # Calculate the angle for this ray
            angle = msg.angle_min + i * msg.angle_increment
            
            # Keep only the rays outside the filtered area
            if angle < min_angle_rad or angle > max_angle_rad:
                ranges.append(msg.ranges[i])
                if len(msg.intensities) > 0:
                    intensities.append(msg.intensities[i])
            else:
                # For filtered rays, set to infinity (or max range)
                ranges.append(float('inf'))
                if len(msg.intensities) > 0:
                    intensities.append(0.0)
        
        filtered_scan.ranges = ranges
        filtered_scan.intensities = intensities
        
        # Publish the filtered scan
        self.publisher.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    
    lidar_filter_node = LidarFilterNode()
    
    try:
        rclpy.spin(lidar_filter_node)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_filter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()