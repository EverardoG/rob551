#!/usr/bin/env python3

# Everardo Gonzalez, gonzaeve@oregonstate.edu
#
# This uses the lidar to build a map

import rospy
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

from lib import *

WORLD_DIM = np.array([8,8]) # meter x meter
CELL_WIDTH = 0.10 # meter
GRID_SIZE = (WORLD_DIM / CELL_WIDTH).astype(int)

class MapBuilder():
    def __init__(self) -> None:
        rospy.init_node('MapBuilder', argv=sys.argv)

        # Subscriber for pose
        self.pose_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odomCallback, queue_size=10)

        # Subscriber for laser scan
        self.lidar_sub = rospy.Subscriber('base_scan', LaserScan, self.scanCallback, queue_size=10)

        # Publisher to visualize map in RVIZ with square markers
        self.points_pub = rospy.Publisher('points', MarkerArray, queue_size=360)
        self.obstacle_pub = rospy.Publisher('obs_points', MarkerArray, queue_size=360)

        # Initialize the map with 0.5 for all the values
        # Need actual size of world that we're going to be updating
        # Need to know how far we are discretizing

        # print("size:",WORLD_DIM/GRID_SIZE)

        self.grid = np.ones(GRID_SIZE)*0.5

        self.robot_position = None # x,y
        self.robot_angle = None # theta relative to world frame
        # Points detected in robot frame
        self.lidar_points_rf = None
        # Points detected in world frame
        self.lidar_points_wf = None

    def odomCallback(self, odom_msg):
        # Update pose of the robot
        self.robot_position, self.robot_angle = extract2DPose(pose_msg)

    def scanCallback(self, scan_msg):
        # Update scan info from lidar
        self.lidar_points_rf = processRawScanMsg(scan_msg)
        # Transform points from robot frame to world frame
        self.lidar_points_wf = transformToWorldFrame(self.lidar_points_rf, self.robot_position, self.robot_angle)

    def update(self):
        # Actually update the map using bayes rule
        # Maybe look at program from ethics class for this?
        # Might have to do some ray tracing...

        # For each detected point
        for pt in self.lidar_points_wf:
            # 
            
            # Calculate a unit vector from the point to the robot's position
            # Iteratively step forward by a cell width until you pass the robot's position


            # We also have the pose of the robot in the world frame
            # Draw a line from the robot to the detected point
            # Make a unit vector from the detected point back to the robot's position


            # Start at the target point
                # Check what part of the grid this is
                # Update that grid cell w. bayes rule
                # Go back by a square width and repeat the check. Update accordingly
                # Stop once we reach our target again (or realistically, overshoot it)
                # Assumption (not true with stage simulator): If the robot occupies a pose, then that position is not occupied
    
        # Publish the new map as a markerarray of squares lol OR DONT
        # Just keep going for a while
        # Maybe every 100 iterations or so output an image of the map
        

        pass

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == "__main__":
    map_builder = MapBuilder()
    map_builder.run()
