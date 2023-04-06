import rospy

from visualization_msgs.msg import Marker, MarkerArray

def buildMarker(x, y, _id, namespace, marker_type, color, timestamp, delete=False):
    marker = Marker()
    marker.header.frame_id = "base_scan"
    # Rename for stage_osu
    # marker.header.frame_id = "base_laser_link"
    marker.header.stamp = timestamp
    marker.ns = namespace
    marker.id = _id
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