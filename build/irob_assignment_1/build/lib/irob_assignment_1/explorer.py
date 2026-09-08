#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import numpy as np
import logging



class ExplorerNode(Node):
    def __init__(self):
        # TODO: Initialize the node with name 'explorer'.
        # TODO: Create a subscription to the occupancy grid map topic (e.g., '/map'),
        #       with queue size 10, and register self.map_callback.
        # TODO: Create an ActionClient for Nav2's NavigateToPose on 'navigate_to_pose'.
        # TODO: Create a set to store visited frontier cells (row, col).
        # TODO: Initialize storage for the latest map (self.map_data = None).
        # TODO: Create a TF2 Buffer and TransformListener (self.tf_buffer, self.tf_listener).
        # TODO: Initialize the robot's grid position (self.robot_position), to be
        #       updated by localization (placeholder values are fine for now).
        # TODO: Initialize a state flag (self.navigating)
        # TODO: Create a periodic timer (e.g., every 5.0 seconds) that calls self.explore().
        
        # Initialization with the name 'explorer'
        super().__init__('explorer')

        # Subscription to map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription 
        
        # Action client creation
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')
        
        # Create a set to store visited frontier cells
        self.visited = set()

        # Initialize storage for the latest map
        self.map_data = None

        # Initialize buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize robot's grid positions 
        self.robot_position = (0,0)

        # Initialize state flag
        self.navigating = False

        # Create timer
        self.timer = self.create_timer(5.0, self.explore)


        

    def map_callback(self, msg):
        """
        TODO: Store the latest occupancy grid message in self.map_data.
              Optionally log that a new map was received.
        """
       
        self.map_data = msg
        pass

    def navigate_to(self, x, y):
        """
        TODO: Construct a PoseStamped goal in the 'map' frame at coordinates (x, y).
              - Set header.stamp using the node clock.
              - Set orientation (e.g., w = 1.0).
        TODO: Wrap that in a NavigateToPose.Goal and send it with the ActionClient.
        TODO: Wait for the action server to be available.
        TODO: Attach self.goal_response_callback to the future returned by send_goal_async().
        """
        self._action_client.wait_for_server()
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg


        self.navigating = True

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        TODO: Check whether the goal was accepted or rejected.
        TODO: If accepted, request the result and attach self.navigation_complete_callback
              to the result future (goal_handle.get_result_async()).
        TODO: Update self.navigating accordingly.
        """
        goal_handle = future.result()
        accepted = goal_handle.accepted
        if accepted:
            self.get_logger().info("The goal was accepted")
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.navigation_complete_callback)
        else:
            self.navigating = False
            self.get_logger().info("The goal was rejected")
        


    def navigation_complete_callback(self, future):
        """
        TODO: Handle completion of the navigation action.
              - If successful, log/record the result.
              - On failure, log the exception or error.
        TODO: Update self.navigating
        """
        self.navigating = False        
        exception = future.exception()
        if exception is not None:
            self.get_logger().error(f'Navigation action failed with exception: {exception}')
            return
        
        result = future.result()
        status = result.status
        self.get_logger().info(f'The status is: {status}')

        if status == 4: # Success
            self.get_logger().info('Goal succeeded')
            # Optional: Mark the current cell/area as successfully explored

        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')

    def find_frontiers(self, map_array):
        """
        TODO: Detect frontier cells in the occupancy grid.
              Definition: a frontier is a FREE cell (value == 0)
              that has at least one UNKNOWN neighbor (value == -1)
              in its 8-neighborhood.

              Steps:
              - Iterate over interior cells (avoid the outermost border).
              - For each free cell, check the 3x3 neighborhood.
              - If any neighbor is unknown, add (row, col) to a list of frontiers.
              - Return the list of (row, col) frontier cells.
        """
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0: 
                    neighborhood = map_array[r - 1 : r + 2, c - 1 : c + 2] # +2 because upper bound is exclusive
                    if np.any(neighborhood == -1):
                        frontiers.append((r, c))
        return frontiers

    def choose_frontier(self, frontiers):
        """
        TODO: Choose the closest frontier to the robot's current grid position.
              - Skip frontiers already present in self.visited_frontiers.
              - Compute Euclidean distance in grid coordinates.
              - Keep the minimum; return (row, col) or None if none available.
              - Add the chosen frontier to self.visited_frontiers.
        """
        closest_frontier = None
        min_distance = float('inf')
        robot_r, robot_c = self.robot_position

        for r, c in frontiers:
            if (r, c) in self.visited:
                continue

            dist = np.sqrt((r - robot_r)**2 + (c - robot_c)**2)
            if dist < min_distance:
                min_distance = dist
                closest_frontier = (r, c)

        if closest_frontier is not None:
            self.visited.add(closest_frontier)

        return closest_frontier

    def get_robot_position(self):
        """
        TODO: Find the robots current position using TF2.
              - Look up the transform from 'map' to 'base_link'.
              - Convert the resulting world (x, y) into grid (row, col) using the
                map's resolution and origin (the inverse of the formula used in
                explore() to go from grid to world).
              - Update self.robot_position.
        """
        if self.map_data is None:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time()
            )
            world_x = t.transform.translation.x
            world_y = t.transform.translation.y

            res = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y

            col = int((world_x - origin_x) / res)
            row = int((world_y - origin_y) / res)
            self.robot_position = (row, col)
        except Exception as e:
            self.get_logger().warn(f'Could not get robot position from TF: {e}')

    def explore(self):
        """
        TODO: Main exploration routine:
              - If no map yet, return early.
              - If self.navigating indicates ongoing navigation, return early.
              - Convert the flat map data to a 2D numpy array with shape
                (height, width).
              - Call find_frontiers(map_array) to get candidates.
              - If none, optionally log "exploration complete" and return.
              - Call get_robot_position().
              - Call choose_frontier(frontiers); if None, return.
              - Convert the chosen (row, col) to world (x, y) using:
                    x = col * resolution + origin.position.x
                    y = row * resolution + origin.position.y
              - Call navigate_to(x, y).
        """
        if self.map_data is None or self.navigating:
            return

        # Convert occupancy grid data to 2D numpy array
        width = self.map_data.info.width
        height = self.map_data.info.height
        map_array = np.array(self.map_data.data)
        map_array = map_array.reshape((height, width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info('No frontiers remain. Exploration complete')
            return

        self.get_robot_position()
        self.get_logger().info(f'The current robot position: {self.robot_position}')
        chosen = self.choose_frontier(frontiers)
        if chosen is None:
            return

        row, col = chosen
        res = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        target_x = col * res + origin.x
        target_y = row * res + origin.y

        self.get_logger().info(f'Navigating to frontier at grid ({row}, {col})')
        self.navigate_to(target_x, target_y)


def main(args=None):
    # TODO: Standard ROS 2 boilerplate:
    # - Initialize rclpy
    # - Create the ExplorerNode
    # - Spin
    # - On shutdown, destroy node and call rclpy.shutdown()
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
