#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String 
from geometry_msgs.msg import PointStamped 
from datetime import datetime
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros
from scipy.spatial.transform import Rotation as R
import json

marker_pub = rospy.Publisher('point_marker', Marker, queue_size=10)

# Create a marker message
marker = Marker()
marker.header.frame_id = "map"
marker.ns = "apriltag"
marker.id = 0
marker.type = Marker.POINTS
marker.action = Marker.ADD
marker.pose.orientation.w = 1.0
marker.scale.x = 0.1
marker.scale.y = 0.1
marker.scale.z = 0.1
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 0.0
marker.color.b = 0.0

filepath = None
listOfTags = []

tags_string = None
tf_listener = None
tf_buffer = None

def detection_callback(data):
    global tags_string
    tags_string = data.data
    # print(tags_string)

def get_transformation_matrix(TF_to, TF_from):
        global tf_buffer

        try:
            pose = tf_buffer.lookup_transform(TF_to, TF_from,rospy.Time())
            #print(pose)
            # extract translation and quaternion from tf pose.
            transformT = [pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z]
            transformQ = (
                pose.transform.rotation.x,
                pose.transform.rotation.y,
                pose.transform.rotation.z,
                pose.transform.rotation.w)
            
            # get equiv rotation matrix from quaternion.
            r = R.from_quat(transformQ).as_matrix()

            # make affine matrix for transformation.
            T = np.array([[r[0][0],r[0][1],r[0][2],transformT[0]],
                            [r[1][0],r[1][1],r[1][2],transformT[1]],
                            [r[2][0],r[2][1],r[2][2],transformT[2]],
                            [0,0,0,1]])
            #print(T)
            return (1,T)
        except Exception as e:
            print("Transform from " + TF_from + " to " + TF_to + " not found.")
            print("Exception: ", e)
            return (0,np.eye(4))


def main():
    global tf_listener, tf_buffer, listOfTags
    rospy.init_node('tag_tracking_node')
    rospy.Subscriber("/apriltag_detections", String, detection_callback)

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Dictionary of transformations - Tag to Odom

    dict_tag_to_odom = {}

    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        
        # Get new Odom to Map for current instance
        try:
            check, odom_to_map = get_transformation_matrix('map', 'odom')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        ###############################################################
        ##  DICTIONARY UPDATION STEP
        ###############################################################

        # Get number of detected tags
        if (tags_string):
            listOfTags = tags_string.split()
        
        #print(listOfTags)

        for tag in listOfTags:

            check, tag_to_odom = get_transformation_matrix('odom',tag)
            if(check):
                dict_tag_to_odom[tag] = tag_to_odom
            

        #print(dict_tag_to_odom)
        ###############################################################
        ##  DICTIONARY ITERATION STEP
        ###############################################################

        points_to_publish = []
        for tag in dict_tag_to_odom.keys():
            
            tag_to_map = odom_to_map @ dict_tag_to_odom[tag]
            
            location = tag_to_map[0:3, 3]

            #print(location)
            points_to_publish.append(Point(location[0], location[1], location[2]))

        marker.points = points_to_publish
        marker_pub.publish(marker)
            # points_to_publish.append(point_msg)
            #point_pub.publish(point_msg)

        # if (points_to_publish):
        #     point_pub.publish(points_to_publish)
        #print(dict_tag_to_odom)
        rate.sleep()

    # json_string = json.dumps(points_to_publish)
    # with open('points_on_map.txt', 'w') as f:
    #     f.write(json_string)
        		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

