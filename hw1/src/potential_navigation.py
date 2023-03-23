#!/usr/bin/env python3

# Everardo Gonzalez, gonzaeve@oregonstate.edu
#
# This example uses potential fields to navigate a mobile robot

import rospy
import sys

import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

def buildMarker(x, y, id, namespace, marker_type, color, timestamp, delete=False):
    marker = Marker()
    marker.header.frame_id = "base_scan"
    # Rename for stage_osu
    # marker.header.frame_id = "base_laser_link"
    marker.header.stamp = timestamp
    marker.ns = namespace
    marker.id = id
    marker.type = marker_type
    marker.action = Marker.ADD
    if delete: marker.action = Marker.DELETE
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 0.9 # Don't forget to set the alpha!
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    return marker

def buildMarkerArray(pts, namespace, marker_type=Marker.SPHERE, color=(0.0,0.0,1.0), timestamp = None):
    timestamp = rospy.Time.now()
    marker_array = MarkerArray()
    id = 0
    for id, (x,y) in enumerate(pts):
        marker_array.markers.append(buildMarker(x,y,id, namespace, marker_type, color, timestamp))
    if id != 0: id += 1
    while id < 360:
        marker_array.markers.append(buildMarker(0,0,id,namespace, marker_type, color, timestamp, delete=True))
        id += 1
    return marker_array

class Navigator():
    def __init__(self) -> None:
        rospy.init_node('Navigator', argv=sys.argv)

        # Setup publisher for velocity commands
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Setup publisher for visualizing what the robot is thinking
        self.points_pub = rospy.Publisher('points', MarkerArray, queue_size=360)
        self.obstacle_pub = rospy.Publisher('obs_points', MarkerArray, queue_size=360)

        # Setup subscriber for lidar data
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.scanCallback, queue_size=10)

        # Subscriber for stage osu
        # self.lidar_sub = rospy.Subscriber('base_scan', LaserScan, self.scanCallback, queue_size=10)

        # self.rep_param = 0.001 
        # self.obstacle_radius = 0.25

        # Parameters for dance demo
        self.rep_param = 0.01
        self.obstacle_radius = 0.5

        # Internal representation of where the robot is. Assume we start at the origin
        self.odom_timestamp = rospy.Time.now()
        self.location = np.array([0,0])

    def odomCallback(self, odom):
        """Grab the odometry of the robot and update the robot location."""
        self.location = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        self.odom_timestamp = odom.header.stamp

    def scanCallback(self, scan):
        """Grab the scan from the lidar. Update actions of robot"""
        pts = self.transformScan(scan)

        # Turn sensed pts into visualizable marker array and visualize it
        pts_marker_array = buildMarkerArray(pts, namespace="pts", marker_type=Marker.SPHERE, color=(0.0,0.0,1.0), timestamp=self.odom_timestamp)
        self.points_pub.publish(pts_marker_array)

        nav_vector = self.calculateNavigationVector(pts)
        x, theta = self.calculateVelocity(nav_vector)
        t = self.generateTwistMsg(x, theta)
        self.vel_pub.publish(t)

    def calculateNavigationVector(self, pts):
        """Calculate vector for navigation based on sensed pts """

        distances = np.linalg.norm(pts, axis=1)

        # Filter out points and distances that are far away from the obstacle
        # outside of the Q* (obstacle_radius) threshold
        # valid_bool = distances <= self.obstacle_radius
        valid_bool = np.logical_and(distances <= self.obstacle_radius, distances > 0.01)
        filtered_distances = distances[valid_bool]
        filtered_pts = pts[valid_bool]

        filtered_marker_array = buildMarkerArray(filtered_pts, namespace="obstacle_pts", marker_type=Marker.CUBE, color=(1.0,0.0,0.0), timestamp=self.odom_timestamp)
        self.obstacle_pub.publish(filtered_marker_array)

        # Calculate repulsion vector if nearby obstacles were detected
        if filtered_pts.shape[0] > 0:
            # Each point is a small repulsion vector
            # Note: There is a gradient in this equation that I'm not including because I can't figure out what it is with respect to
            # It seems like it's gradient of distance wrt distance... which is just 1
            rep_strengths = self.rep_param * (1/self.obstacle_radius - 1/filtered_distances) * (1/filtered_distances)**2 
            rep_norms = np.linalg.norm(filtered_pts, axis=1)
            rep_norms_expanded = np.array([rep_norms, rep_norms]).T
            rep_unit_vecs = filtered_pts / rep_norms_expanded

            rep_strengths_expand = np.expand_dims(rep_strengths, axis=1)
            rep_vecs = rep_strengths_expand*rep_unit_vecs

            repulsion_vector = np.sum(rep_vecs, axis=0)

        else:
            repulsion_vector = np.array([0.,0.])

        return repulsion_vector        
        
    def calculateVelocity(self, navigation_vector):
        """Turn the navigation vector into a x, theta velocity"""
        x = 0.1
        theta = np.arctan2(navigation_vector[1], navigation_vector[0])
        return x, theta

    def generateTwistMsg(self, x, theta):
        """Turn the velocities into a twist message"""
        t = Twist()
        t.linear.x = x
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = theta
        return t

    def transformScan(self, scan):
        # Turn scan into x,y points
        distances = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(distances))

        # Filter out inf distances
        not_inf_bool = np.logical_not(np.isinf(distances))
        filtered_distances = distances[not_inf_bool]
        filtered_angles = angles[not_inf_bool]

        # Turn these filtered distances into xs and ys
        x = np.array(filtered_distances)*np.cos(filtered_angles)
        y = np.array(filtered_distances)*np.sin(filtered_angles)

        # Turn xs and ys into points array
        pts = np.array([x,y]).T

        return pts


if __name__ == '__main__':
    navigator = Navigator()
    rospy.spin()
