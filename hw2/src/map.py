#!/usr/bin/env python3

# Everardo Gonzalez, gonzaeve@oregonstate.edu
#
# This uses the lidar to build a map as an occupancy grid

import rospy
import sys

import numpy as np

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

from transform_utils import processRawScanMsg, extract2DPose, transformToWorldFrame, distance

# Configuration parameters for the map
WORLD_DIM = np.array([8,8]) # meter x meter
CELL_WIDTH = 0.10 # meter
GRID_SIZE = (WORLD_DIM / CELL_WIDTH).astype(int)

# Configuration parameters for the bayes filter
# True positive rate
LIDAR_TPR = 0.9
# True negative rate
LIDAR_TNR = 0.9
# False negative rate. Inverse of true positive rate
LIDAR_FNR = 1 - LIDAR_TPR
# Fale positive rate. Inverse of true negative rate
LIDAR_FPR = 1 - LIDAR_TNR

class MapBuilder():
    def __init__(self) -> None:
        rospy.init_node('MapBuilder', argv=sys.argv)

        # Grid is x,y. Indexed by self.grid[y][x]
        self.grid = np.ones(GRID_SIZE)*0.5

        self.robot_position = None # x,y
        self.robot_angle = None # theta relative to world frame
        # Points detected in robot frame
        self.lidar_points_rf = None
        # Points detected in world frame
        self.lidar_points_wf = None

        # Subscriber for pose
        self.pose_sub = rospy.Subscriber('base_pose_ground_truth', Odometry, self.odomCallback, queue_size=10)

        # Subscriber for laser scan
        self.lidar_sub = rospy.Subscriber('base_scan', LaserScan, self.scanCallback, queue_size=10)

        # Publisher to visualize map in RVIZ with square markers
        self.points_rf_pub = rospy.Publisher('points_robot_frame', MarkerArray, queue_size=360)
        self.points_wf_pub = rospy.Publisher('points_world_frame', MarkerArray, queue_size=360)
        self.map_pub = rospy.Publisher('occupancy_grid', MarkerArray, queue_size=self.grid.size)

    def odomCallback(self, odom_msg):
        # Update pose of the robot
        self.robot_position, self.robot_angle = extract2DPose(odom_msg)

    def scanCallback(self, scan_msg):
        # Update scan info from lidar
        self.lidar_points_rf = processRawScanMsg(scan_msg)
        # Transform points from robot frame to world frame
        self.lidar_points_wf = transformToWorldFrame(self.lidar_points_rf, self.robot_position, self.robot_angle)

    def binPoint(self, pt):
        """Figure out which grid cell this point belongs to"""
        # Scale point by cell width
        scaled_pt = pt/CELL_WIDTH
        # Just round down to get the integer position in the bin
        bin_coord = scaled_pt.astype(int)
        return bin_coord

    def updateProbCell(self, bin_coord, detected):
        """Update probability an obstacle is in this bin"""
        prior = self.grid[bin_coord[1]][bin_coord[0]] 
        if detected:
            likelihood = LIDAR_TPR*prior + LIDAR_FPR*(1-prior)
            new_prior = (LIDAR_TPR*prior) / likelihood
        else:
            likelihood = LIDAR_FNR*prior + LIDAR_TNR*(1-prior)
            new_prior = (LIDAR_FNR*prior) / likelihood
        self.grid[bin_coord[1]][bin_coord[0]] = new_prior

    def update(self):
        # Actually update the map using bayes rule
        # Maybe look at program from ethics class for this?
        # Might have to do some ray tracing...

        if self.lidar_points_wf is None or self.robot_position is None or self.robot_angle is None:
            return None

        # For each detected point
        for pt in self.lidar_points_wf:
            # Figure out which grid cell it is
            bin_coord = self.binPoint(pt)

            # Update probability for that bin
            self.updateProbCell(bin_coord, detected=True)

            # Calculate a unit vector from the point to the robot's position
            vec = self.robot_position - pt
            unit_vec = vec/np.linalg.norm(vec)

            # Now turn that into a step vector by scaling by cell width
            step_vec = unit_vec * CELL_WIDTH

            # Iteratively step forward by a cell width until you pass the robot's position
            check_pt = np.copy(pt) # Point we are currently checking
            while distance(check_pt, self.robot_position) > CELL_WIDTH/2: 
                #TODO: Make sure CELL_WIDTH/2 makes sense here to not cause overshoot but still go through all important grid cells
                # Step the check point forward towards (or technically one time "past") the robot
                check_pt += step_vec
                # Update the probability of the cell we're in
                check_bin_coord = self.binPoint(check_pt)
                self.updateProbCell(check_bin_coord, detected=False)
                # Assumption (not true with stage simulator): If the robot occupies a pose, then that position is not occupied

        # Publish the points

        # Publish the new map as a markerarray of squares

        # Just keep going for a while
        # Maybe every 100 iterations or so output an image of the map

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == "__main__":
    map_builder = MapBuilder()
    map_builder.run()
