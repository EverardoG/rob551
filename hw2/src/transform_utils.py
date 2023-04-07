import numpy as np
from scipy.spatial.transform import Rotation

def processRawScanMsg(scan_msg):
    # Process all of the lidar points
    # Filter out any points where nothing was actually detected (Or if there are infs or something weird like that)

    # Turn scan into x,y points
    distances = np.array(scan_msg.ranges)
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(distances))

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

def extract2DPose(odom_msg):
    # Get position
    position_msg = odom_msg.pose.pose.position
    position = np.array([position_msg.x, position_msg.y])
    
    # Grab the orientation out of the odometry message as a quaternion
    orientation_msg = odom_msg.pose.pose.orientation
    quaternion = np.array([orientation_msg.w, orientation_msg.x, orientation_msg.y, orientation_msg.z])

    # Turn the quaternion into an angle in a 2D plane (relative to world frame) using scipy rotation library
    scipy_rotation = Rotation.from_quat(quaternion)
    euler_angles = scipy_rotation.as_euler('zyx', degrees=False)

    # Return the position and angle in 2D plane
    return position, euler_angles[2]+np.pi

def transformToWorldFrame(points_rf, robot_position, robot_angle):
    # Construct rotation matrix from robot position and angle in world frame
    theta = robot_angle
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    # Apply rotation matrix
    rotated_pts = points_rf.dot(R)
    # Apply translation w. simple addition
    transformed_pts = rotated_pts + robot_position
    
    return transformed_pts

def distance(pt1, pt2):
    """Calculate the distance between two points"""
    return np.linalg.norm(pt1-pt2)
