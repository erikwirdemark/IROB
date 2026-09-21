#!/usr/bin/env python3

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Python standard library
import copy

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import message_filters

# ROS 2 messages
from geometry_msgs.msg import PoseStamped as PoseStampedROS
from sensor_msgs.msg import LaserScan as LaserScanROS
from nav_msgs.msg import OccupancyGrid as OccupancyGridROS
from nav_msgs.msg import Odometry as OdometryROS
from map_msgs.msg import OccupancyGridUpdate as OccupancyGridUpdateROS
from geometry_msgs.msg import Pose

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped
from local.sensor_msgs import LaserScan
from local.nav_msgs import OccupancyGrid
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap
from mapping import Mapping


class EngineROS2(Node):
    def __init__(self, map_frame_id, map_resolution, map_width, map_height,
                 map_origin_x, map_origin_y, map_origin_yaw, inflate_radius,
                 unknown_space, free_space, c_space, occupied_space, optional=None):
        super().__init__('Mapper')
        
        self.__pose = None
        self.__map = GridMap(map_frame_id, map_resolution, map_width, map_height,
                         map_origin_x, map_origin_y, map_origin_yaw)
        
        self.__inflated_map = self.__map

        self.__mapping = Mapping(unknown_space, free_space, c_space,
                                 occupied_space, inflate_radius, optional)
        
        # Subscribers
        self.__odom_sub = message_filters.Subscriber(self, OdometryROS, 'odom')
        self.__scan_sub = message_filters.Subscriber(self, LaserScanROS, 'scan')

        # Time Synchronizer
        self.__ts = message_filters.ApproximateTimeSynchronizer([self.__odom_sub,
                                                                 self.__scan_sub], 10, 0.01)
        self.__ts.registerCallback(self.callback)

        # QoS Profiles
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        default_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Publishers
        self.__map_pub = self.create_publisher(OccupancyGridROS, 'map', latched_qos)

        self.__map_updates_pub = self.create_publisher(OccupancyGridUpdateROS, "map_updates", default_qos)
        
        self.__map_inflated_pub = self.create_publisher(OccupancyGridROS, 'inflated_map', latched_qos)

        self.publish_map()
        
        rclpy.spin(self)

    def callback(self, odom_ros, scan_ros):
        scan = self.from_ros_scan(scan_ros)
        pose = self.from_ros_odom(odom_ros)

        self.__map, update = self.__mapping.update_map(self.__map, pose, scan)

        if isinstance(update, OccupancyGridUpdate) and len(update.data) != 0:
            # self.publish_map_update(update)
            self.publish_map()
            map = copy.deepcopy(self.__map)
            self.__inflated_map = self.__mapping.inflate_map(map)
            self.publish_inflated_map()
        else:
            self.publish_map()

    def publish_map(self):
        # Get ROS occupancy map
        map = self.map_to_message(self.__map)
        self.__map_pub.publish(map)

    def publish_map_update(self, update):

        try:
            # Get ROS occupancy map update
            update_ros = self.map_update_to_message(update)
            # Only send out the update
            self.__map_updates_pub.publish(update_ros)
            self.get_logger().info(f"Published update: {update_ros.width}x{update_ros.height} at ({update_ros.x},{update_ros.y})")
        except Exception as e:
            self.get_logger().error(f"Error publishing update: {str(e)}")

    def publish_inflated_map(self):
        # Get ROS occupancy map
        map = self.map_to_message(self.__inflated_map)
        self.__map_inflated_pub.publish(map)

    def from_ros_scan(self, scan_ros):
        scan = LaserScan()

        scan.header.stamp = scan_ros.header.stamp
        scan.header.frame_id = scan_ros.header.frame_id

        scan.angle_min = scan_ros.angle_min
        scan.angle_max = scan_ros.angle_max
        scan.angle_increment = scan_ros.angle_increment
        scan.time_increment = scan_ros.time_increment
        scan.range_min = scan_ros.range_min
        scan.range_max = scan_ros.range_max
        scan.ranges = scan_ros.ranges
        scan.intensities = scan_ros.intensities

        return scan

    def from_ros_odom(self, odom_ros):
        pose_ros = PoseStampedROS()
        pose_ros.header = odom_ros.header
        pose_ros.pose = odom_ros.pose.pose

        pose = PoseStamped()

        pose.header.stamp = pose_ros.header.stamp
        pose.header.frame_id = pose_ros.header.frame_id

        pose.pose.position.x = pose_ros.pose.position.x
        pose.pose.position.y = pose_ros.pose.position.y
        pose.pose.position.z = pose_ros.pose.position.z

        pose.pose.orientation.x = pose_ros.pose.orientation.x
        pose.pose.orientation.y = pose_ros.pose.orientation.y
        pose.pose.orientation.z = pose_ros.pose.orientation.z
        pose.pose.orientation.w = pose_ros.pose.orientation.w

        return pose
    
    def pose_to_message(self, pose):

        pose_ros = Pose()

        pose_ros.position.x = float(pose.position.x)
        pose_ros.position.y = float(pose.position.y)
        pose_ros.position.z = float(pose.position.z)

        pose_ros.orientation.x = float(pose.orientation.x)
        pose_ros.orientation.y = float(pose.orientation.y)
        pose_ros.orientation.z = float(pose.orientation.z)
        pose_ros.orientation.w = float(pose.orientation.w)

        return pose_ros

    def map_to_message(self, map):
        '''
        :type map: Map
        '''
        map = map.to_message()  # OccupancyGrid
        map_ros = OccupancyGridROS()

        # Fill in the header
        map_ros.header.stamp = self.get_clock().now().to_msg()
        map_ros.header.frame_id = map.header.frame_id

        # Fill in the info
        map_ros.info.resolution = map.info.resolution
        map_ros.info.width = map.info.width
        map_ros.info.height = map.info.height

        map_ros.info.origin = self.pose_to_message(map.info.origin)

        # Fill in the map data
        # map_ros.data = map.data
        map_ros.data = [int(max(-128, min(127, x))) for x in map.data]

        return map_ros

    def map_update_to_message(self, update):
        '''
        :type update: OccupancyGridUpdate
        '''
        update_ros = OccupancyGridUpdateROS()

        # Fill in the header
        update_ros.header.stamp = self.get_clock().now().to_msg()
        update_ros.header.frame_id = "odom"

        update_ros.x = int(update.x)
        update_ros.y = int(update.y)
        update_ros.width = update.width
        update_ros.height = update.height
        update_ros.data = update.data

        return update_ros
