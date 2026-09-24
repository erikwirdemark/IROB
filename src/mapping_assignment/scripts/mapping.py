#!/usr/bin/env python3

"""
    # Erik Wirdemark
    # erikwir
    # erikwir@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy

import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """

        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        range_min = scan.range_min
        range_max = scan.range_max
        ranges = scan.ranges

        position_x = pose.pose.position.x
        position_y = pose.pose.position.y
        origin_x, origin_y = origin.position.x, origin.position.y


        ranges = np.array(ranges)
        # Discards ranges out of bounds (Should I clip instead maybe?)
        #ranges = np.where((ranges > range_min) & (ranges < range_max), ranges, None)
        points = []
        for i in range(len(ranges)):
            if np.isnan(ranges[i]) or np.isinf(ranges[i]):
                continue
            if (ranges[i] > range_min) & (ranges[i] < range_max):
                angle = angle_min + i * angle_increment + robot_yaw
                distance = ranges[i]
                x = position_x + distance * cos(angle)
                y = position_y + distance * sin(angle)
                points.append((x, y, distance))
        #(ranges > range_min) & (ranges < range_max) order by distance to avoid assigning free space to occupied space (as the presentation mentions)
        points.sort(key=lambda point: point[2])
        occupied_cells = []
        robot_x_cell = int((position_x - origin_x) / resolution)
        robot_y_cell = int((position_y - origin_y) / resolution)
        min_x_cell = robot_x_cell
        min_y_cell = robot_y_cell
        max_x_cell = robot_x_cell
        max_y_cell = robot_y_cell

        for point in points:
            x, y, _ = point
            x_cell = int((x - origin_x) // resolution)
            y_cell = int((y - origin_y) // resolution)
            occupied_cells.append((x_cell, y_cell))
            traversed = self.raytrace((int(robot_x_cell), int(robot_y_cell)), (x_cell, y_cell))
            for cx, cy in traversed:
                if self.is_in_bounds(grid_map, cx, cy):
                    self.add_to_map(grid_map, cx, cy, self.free_space)
            self.add_to_map(grid_map, x_cell, y_cell, self.occupied_space)

            min_x_cell, min_y_cell, max_x_cell, max_y_cell = update_bounding_box(min_x_cell, min_y_cell, max_x_cell, max_y_cell, x_cell, y_cell)
        
        """
        For C only!
        Fill in the update correctly below.
        """ 

        min_x_cell = max(0, min_x_cell)
        min_y_cell = max(0, min_y_cell)
        max_x_cell = min(grid_map.get_width() - 1, max_x_cell)
        max_y_cell = min(grid_map.get_height() - 1, max_y_cell)

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_x_cell
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_y_cell
        # Maximum x index - minimum x index + 1
        update.width = max_x_cell - min_x_cell + 1
        # Maximum y index - minimum y index + 1
        update.height = max_y_cell - min_y_cell + 1
        # The map data inside the rectangle, in row-major order.
        update_data = []
        for y in range(update.y, update.y + update.height):
            for x in range(update.x, update.x + update.width):
                update_data.append(grid_map[x, y])
        update.data = update_data
        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        occupied_cells = []
        for x in range(grid_map.get_width()):
            for y in range(grid_map.get_height()):
                if grid_map[x, y] == self.occupied_space:
                    occupied_cells.append((x, y))
        offsets = [
            (dx, dy) 
            for dx in range(-self.radius, self.radius + 1) 
            for dy in range(-self.radius, self.radius + 1) 
            if dx * dx + dy * dy <= self.radius * self.radius
        ]
        for x, y in occupied_cells:
            for dx, dy in offsets:
                new_x = x + dx
                new_y = y + dy
                if self.is_in_bounds(grid_map, new_x, new_y) and grid_map[new_x, new_y] != self.occupied_space:
                    self.add_to_map(grid_map, new_x, new_y, self.c_space)

        return grid_map

def update_bounding_box(min_x, min_y, max_x, max_y, x, y):
    if x < min_x:
        min_x = x
    if y < min_y:
        min_y = y
    if x > max_x:
        max_x = x
    if y > max_y:
        max_y = y
    return min_x, min_y, max_x, max_y
