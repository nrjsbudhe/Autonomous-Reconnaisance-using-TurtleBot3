
import rospy
import tf.transformations as tf
from visualization_msgs.msg import MarkerArray

# Define the time-stamp at which to get the pose


# Initialize ROS node
rospy.init_node('get_robot_pose1')

target_time = rospy.Time.now()
# Subscribe to the /trajectory_node_list topic
robot_pose = None
def trajectory_node_list_callback(data):
    global robot_pose
    if robot_pose is not None:
        return

    for marker in data.markers:        	
        # Extract the pose from the marker
        print("i")
        position = marker.pose.position
        orientation = marker.pose.orientation
        euler = tf.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        robot_pose = (position.x, position.y, euler[2])
    if robot_pose is not None:
        rospy.loginfo("Found robot pose: {}".format(robot_pose))
rospy.Subscriber('/trajectory_node_list', MarkerArray, trajectory_node_list_callback)

print("iter")
# Spin the ROS node to process incoming messages
rospy.spin()

