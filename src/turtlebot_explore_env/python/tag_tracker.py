#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String 
from geometry_msgs.msg import PointStamped 
from datetime import datetime

import tf2_ros

from scipy.spatial.transform import Rotation as R

filepath = None
listOfTags = []

tags_string = None
tf_listener = None
tf_buffer = None

def detection_callback(data):
    global tags_string
    tags_string = data.data
    print(tags_string)

def get_transformation_matrix(TF_to, TF_from):
        global tf_buffer

        try:
            pose = tf_buffer.lookup_transform(TF_to, TF_from,rospy.Time())

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
        
            return T
        except Exception as e:
            print("Transform from " + TF_from + " to " + TF_to + " not found.")
            print("Exception: ", e)
            return np.eye(4)


def main():
    global tf_listener, tf_buffer, listOfTags
    rospy.init_node('tag_tracking_node')
    rospy.Subscriber("/apriltag_detections", String, detection_callback)
    point_pub = rospy.Publisher("/tag_map_points", PointStamped, queue_size=10)


    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    

    # Dictionary of transformations - Tag to Odom

    dict_tag_to_odom = {}

    # generate filepath that tags will be written to
    # dt = datetime.now()
    # run_id = dt.strftime("%Y-%m-%d-%H-%M-%S")
    # filepath = "tags_" + str(run_id) + ".txt"

    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        
        # Get new Odom to Map for current instance
        try:
            odom_to_map = tf_buffer.lookup_transform('map', 'odom',rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue


        ###############################################################
        ##  DICTIONARY UPDATION STEP
        ###############################################################

        # Get number of detected tags
        if (tags_string):
            listOfTags = tags_string.split()

        for tag in listOfTags:

            tag_to_odom = get_transformation_matrix('odom',tag)
            dict_tag_to_odom[tag] = tag_to_odom

        ###############################################################
        ##  DICTIONARY ITERATION STEP
        ###############################################################

        points_to_publish = []
        for tag in dict_tag_to_odom.keys():

            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            
            odom_to_map = get_transformation_matrix('map','odom')
            tag_to_map = odom_to_map @ dict_tag_to_odom[tag]
            
            point_msg.point.x = float(tag_to_map[0][2])
            point_msg.point.y = float(tag_to_map[1][2])
            point_msg.point.z = float(tag_to_map[2][2])

            # points_to_publish.append(point_msg)
            point_pub.publish(point_msg)

        # if (points_to_publish):
        #     point_pub.publish(points_to_publish)

        rate.sleep()
        		

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

