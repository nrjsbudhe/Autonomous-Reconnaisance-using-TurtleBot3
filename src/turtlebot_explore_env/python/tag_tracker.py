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
from nav_msgs.msg import Path
from cartographer_ros_msgs.msg import SubmapList, TrajectoryStates

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

dict_tag_to_baselink = {}
dict_baselink_to_map = {}


tags_string = None
tf_listener = None
tf_buffer = None

'''
    get_pose_to_SE3
'''

def get_pose_to_SE3(pose):

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

def get_transformation_matrix(TF_to, TF_from):
        global tf_buffer

        try:
            pose = tf_buffer.lookup_transform(TF_to, TF_from,rospy.Time())
            T = get_pose_to_SE3(pose)
            
            return (1,T)
        
        except Exception as e:
            print("Transform from " + TF_from + " to " + TF_to + " not found.")
            print("Exception: ", e)
            return (0,np.eye(4))

'''
CALLBACK FUNCTIONS
'''
def trajectory_callback(data):
    global dict_tag_to_baselink
    global dict_baselink_to_map

    for tag in dict_tag_to_baselink.keys():
        target_time = dict_tag_to_baselink[tag][1]
        print(target_time)
        # target_time = rospy.Time(123456789.0)
        closest_pose = None
        closest_time_diff = None
        for pose in data.poses:
            print(pose.header.stamp)
            time_diff = abs((pose.header.stamp - target_time).to_sec())
            if closest_pose is None or time_diff < closest_time_diff:
                closest_pose = pose
                closest_time_diff = time_diff
            
        dict_baselink_to_map[tag] = get_pose_to_SE3(closest_pose)
        

def detection_callback(data):
    global tags_string
    tags_string = data.data
    # print(tags_string)




def main():
    global tf_listener, tf_buffer, listOfTags, dict_tag_to_baselink, dict_baselink_to_map
    rospy.init_node('tag_tracking_node')
    rospy.Subscriber("/apriltag_detections", String, detection_callback)
    


    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        
        # # Get new Odom to Map for current instance
        # try:
        #     check, odom_to_map = get_transformation_matrix('map', 'odom')
            
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     continue

        ###############################################################
        ##  Update Dictionary
        ###############################################################

        # Get number of detected tags
        if (tags_string):
            listOfTags = tags_string.split()
        
        #print(listOfTags)

        # Update Tags to Baselink  Dictionary
        for tag in listOfTags:

            check, tag_to_baselink = get_transformation_matrix('base_link',tag)
            if(check):
                dict_tag_to_baselink[tag] = (tag_to_baselink,rospy.get_time())

        ###############################################################
        ##  DICTIONARY ITERATION STEP
        ###############################################################

        points_to_publish = []
        for tag in dict_tag_to_baselink.keys():
            #print(dict_tag_to_baselink[tag][1])
            if tag in dict_baselink_to_map.keys():
                tag_to_map = dict_baselink_to_map[tag] @ dict_tag_to_baselink[tag][0]
            
                location = tag_to_map[0:3, 3]

                #print(location)
                points_to_publish.append(Point(location[0], location[1], location[2]))

        #print(points_to_publish)
        if(points_to_publish):
            marker.points = points_to_publish
            marker_pub.publish(marker)

        rate.sleep()

        		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

