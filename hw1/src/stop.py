#!/usr/bin/env python3

# Everardo Gonzalez, gonzaeve@oregonstate.edu
#
# This example gives the robot callback based driver


# Import ROS Python basic API and sys
import rospy
import sys

import numpy as np
import matplotlib.pyplot as plt


# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


class StopController():
    def __init__(self):
        self.stop_distance = 0.25 # meters
        self.robot_width = 0.75 # meters, approx
        self.x_vel = 0.1

        self.first_callback = True
        

        rospy.init_node('StopController', argv=sys.argv)
        
        # Setup publisher for velocity commands
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Setup subscriber for lidar data
        # self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.scanCallback, queue_size=10)
        # Subscriber for stage_osu
        self.lidar_sub = rospy.Subscriber('base_scan', LaserScan, self.scanCallback, queue_size=10)

    def driveForward(self, x_vel):
        t = Twist()
        t.linear.x = x_vel
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0

        self.vel_pub.publish(t)
    
    def scanCallback(self, scan):
        rospy.loginfo("Recieved lidar scan")
        distances = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(distances))

        # Filter out inf distances
        not_inf_bool = np.logical_not(np.isinf(distances))
        filtered_distances = distances[not_inf_bool]
        filtered_angles = angles[not_inf_bool]

        # Turn these filtered distances into xs and ys
        x = np.array(filtered_distances)*np.cos(filtered_angles)
        y = np.array(filtered_distances)*np.sin(filtered_angles)

        if self.first_callback:
            plt.plot(x, y, '.')
            
        # Turn xs and ys into points array
        pts = np.array([x,y]).T

        # Filter out pts that are not in front of the robot. Technically in front and behind the robot
        in_front_bool = np.logical_and(pts[:,1] < self.robot_width/2 , pts[:,1] >= -self.robot_width/2)
        in_front_pts = pts[in_front_bool]

        if self.first_callback:
            plt.plot(in_front_pts[:,0], in_front_pts[:,1], '+')

        # Filter out pts that are within collision distance
        collision_bool = np.logical_and(in_front_pts[:,0] < self.stop_distance, in_front_pts[:,0] >= 0.1)
        collision_pts = in_front_pts[collision_bool]
        rospy.loginfo(collision_pts.shape)

        if self.first_callback:
            plt.plot(collision_pts[:,0], collision_pts[:,1], 'x')

        if collision_pts.shape[0] > 1:
            # Print out a log message to the INFO channel to let us know it's working.
            rospy.loginfo('Collision imminent!')
            # Stop the robot
            self.driveForward(0)

        else:
            # Keep moving forward
            self.driveForward(self.x_vel)
        
        if self.first_callback:
            plt.xlim([-3,3])
            plt.ylim([-3,3])
            plt.plot([0,0], [0,1], '-')
            plt.plot([0,1], [0,0], '-')
            plt.savefig("lidar_scan.png")
            self.first_callback = False

if __name__ == '__main__':
    stop_controller = StopController()
    rospy.spin()
