#!/usr/bin/env python3
from cgitb import lookup
import rospy
import numpy as np
from std_msgs.msg import String 
from geometry_msgs.msg import PointStamped 
from datetime import datetime
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Path
from cartographer_ros_msgs.srv import TrajectoryQuery
from apriltag_ros.msg import AprilTagDetectionArray

tf_listener = tf.TransformListener()

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

listOfTags = []

dict_tag_to_baselink = []
dict_baselink_to_map = {}

'''
    get_pose_to_SE3
'''

def get_pose_to_SE3(Tx, Ty, Tz, X, Y, Z, W):

    # extract translation and quaternion from tf pose.
    transformT = [Tx, Ty, Tz]
    transformQ = (
        X,
        Y,
        Z,
        W)
    
    # get equiv rotation matrix from quaternion.
    r = R.from_quat(transformQ).as_matrix()

    # make affine matrix for transformation.
    T = np.array([[r[0][0],r[0][1],r[0][2],transformT[0]],
                    [r[1][0],r[1][1],r[1][2],transformT[1]],
                    [r[2][0],r[2][1],r[2][2],transformT[2]],
                    [0,0,0,1]])
    
    return T

def get_transformation_matrix(TF_to, TF_from):
        global tf_listener

        try:
            pose = tf_listener.lookup_transform(TF_to, TF_from,rospy.Time(0))
            lookup_time = pose.header.stamp.to_sec()
            T = get_pose_to_SE3(pose.transform.translation.x,pose.transform.translation.y,pose.transform.translation.z,
                                pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z,pose.transform.rotation.w)
            
            return 1,T,lookup_time
        
        except Exception as e:
            print("Transform from " + TF_from + " to " + TF_to + " not found.")
            print("Exception: ", e)
            return 0,np.eye(4),0

'''
CALLBACK FUNCTIONS
'''
def get_trajectory(response):
    global dict_tag_to_baselink
    global dict_baselink_to_map
    global tf_listener
    
    try:

        for tag in dict_tag_to_baselink.keys():
            t_secs = dict_tag_to_baselink[tag][1]

            if t_secs.to_sec() <= response.trajectory[-1].header.stamp.to_sec():

                for pose_id in range(len(response.trajectory)):
                    POSE = response.trajectory[pose_id]
                    if POSE.header.stamp.to_sec() > t_secs.to_sec():
                        target_pose_R = response.trajectory[pose_id].pose.orientation
                        target_pose_T = response.trajectory[pose_id].pose.position
                        break
            else:
                print("realtime")
                target_pose = tf_listener.lookup_transform('map', 'base_link',rospy.Time(0))
                target_pose_R = target_pose.transform.rotation
                target_pose_T = target_pose.transform.translation

            dict_baselink_to_map[tag] = (target_pose_T,target_pose_R)
            target_pose_T = None
            target_pose_R = None

    except Exception as e:
        print("Transform from " + "map" + " to " + "base_link" + " not found.")
        print("Exception: ", e)
        

def detection_callback(data):

    global dict_tag_to_baselink
    for tag in data.detections:
            dict_tag_to_baselink[tag.id[0]] = (0,tag.pose.header.stamp)


def main():
    global tf_listener, dict_tag_to_baselink, dict_baselink_to_map
    rospy.init_node('tag_tracking_node')
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, detection_callback)
    get_trajectory_query = rospy.ServiceProxy('trajectory_query', TrajectoryQuery)

    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        
        #rospy.wait_for_service('/trajectory_query')
        response = get_trajectory_query(0)
        get_trajectory(response)

        for tag_id in dict_tag_to_baselink.keys():
            try:
                apriltag_wrt_baselink = tf_listener.lookupTransform(f'/tag_{tag_id}','/baselink', rospy.Time(0))
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            dict_tag_to_baselink[tag_id][0] = apriltag_wrt_baselink    
        
        # Publish location of Tags on MarkerArray topic
        points_to_publish = []
        for tag in dict_tag_to_baselink.keys():

            if tag in dict_baselink_to_map.keys():
                
                baselink_to_map = get_pose_to_SE3(dict_baselink_to_map[tag][0].x,dict_baselink_to_map[tag][0].y,dict_baselink_to_map[tag][0].z,dict_baselink_to_map[tag][1].x,dict_baselink_to_map[tag][1].y,dict_baselink_to_map[tag][1].z,dict_baselink_to_map[tag][1].w)
                tag_to_baselink = get_pose_to_SE3(dict_tag_to_baselink[tag][0].position.x,dict_tag_to_baselink[tag][0].position.y,dict_tag_to_baselink[tag][0].position.z,dict_tag_to_baselink[tag][0].orientation.x,dict_tag_to_baselink[tag][0].orientation.y,dict_tag_to_baselink[tag][0].orientation.z,dict_tag_to_baselink[tag][0].orientation.w)
                
                # Generate tag_to_map and Get translational component from tag_to_map
                tag_to_map = baselink_to_map @ tag_to_baselink              
                location = tag_to_map[0:3, 3]

                # Add points to publish to points_to_publish list
                points_to_publish.append(Point(location[0], location[1], location[2]))

        if points_to_publish:
            marker.points = points_to_publish
            marker_pub.publish(marker)

        rate.sleep()

        		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

