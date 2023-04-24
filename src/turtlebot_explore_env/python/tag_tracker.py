#!/usr/bin/env python3
from cgitb import lookup
import rospy
import numpy as np
from std_msgs.msg import String 
#from geometry_msgs.msg import PointStamped 
from datetime import datetime
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros
from scipy.spatial.transform import Rotation as R
#from nav_msgs.msg import Path
from cartographer_ros_msgs.srv import TrajectoryQuery

marker_pub = rospy.Publisher('point_marker', Marker, queue_size=10)
get_trajectory_query = rospy.ServiceProxy('trajectory_query', TrajectoryQuery)

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

dict_tag_to_baselink = {}
dict_baselink_to_map = {}


tags_string = None
tf_listener = None
tf_buffer = None

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

def get_transformation_matrix(TF_to, TF_from, TF_at_time):
        global tf_buffer
      
        try:
            pose = tf_buffer.lookup_transform(TF_to, TF_from, TF_at_time)
            lookup_time = pose.header.stamp
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
def get_trajectory():
    global dict_tag_to_baselink
    global dict_baselink_to_map

    for tag in dict_tag_to_baselink.keys():
        lookup_time = dict_tag_to_baselink[tag][1]
        t_secs = lookup_time.to_sec()

        rospy.wait_for_service('/trajectory_query')
        response = get_trajectory_query(0)
        
        if t_secs <= response.trajectory[-1].header.stamp.to_sec():

            for curr_pose in response.trajectory:
                if curr_pose.header.stamp.to_sec() > t_secs:
                    print("blah blah")
                    target_pose = curr_pose.pose
                    target_pose_SE3 = get_pose_to_SE3(target_pose.position.x,target_pose.position.y,target_pose.position.z,target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w)
                    dict_baselink_to_map[tag] = target_pose_SE3
                    return
        else:
            check, target_pose_SE3, _ = get_transformation_matrix('map','base_link', lookup_time)
            if(check):
                dict_baselink_to_map[tag] = target_pose_SE3
            return
        
        
'''
CALLBACK FUNCTION
'''
def detection_callback(data):
    global tags_string
    tags_string = data.data

def main():
    global tf_listener, tf_buffer, dict_tag_to_baselink, dict_baselink_to_map

    # Initialize Nodes and add subscriber topic
    rospy.init_node('tag_tracking_node')
    rospy.Subscriber("/apriltag_detections", String, detection_callback)

    # Create tf_buffer and tf_listener objects for using transformations from TF tree
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(5.0)
    
    list_of_tags = []
    while not rospy.is_shutdown():
        # Get number of detected tags
        get_trajectory()
        
        if (tags_string):
            list_of_tags = tags_string.split()
        
        print(dict_baselink_to_map)

        # Update Tags to Baselink  Dictionary
        for tag in list_of_tags:
            #if tag not in dict_tag_to_baselink.keys(): 
                check, tag_to_baselink, lookup_time = get_transformation_matrix('base_link', tag, rospy.Time())
                if(check):
                    dict_tag_to_baselink[tag] = (tag_to_baselink,lookup_time)


        # Generate transformation: Tag to Map
        points_to_publish = []

        for tag in dict_tag_to_baselink.keys():
            #print(dict_tag_to_baselink[tag][1])
            if tag in dict_baselink_to_map.keys():
                tag_to_map = dict_baselink_to_map[tag] @ dict_tag_to_baselink[tag][0]
            
                location = tag_to_map[0:3, 3]
                print(tag)
                print("B TO M: \n", dict_baselink_to_map[tag])
                print("T TO B: \n", dict_tag_to_baselink[tag][0])
                points_to_publish.append(Point(location[0], location[1], location[2]))

        # Publish points on MarkerArray Topic
        print(points_to_publish)
        if(points_to_publish):
            marker.points = points_to_publish
            marker_pub.publish(marker)

        rate.sleep()

        		
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

