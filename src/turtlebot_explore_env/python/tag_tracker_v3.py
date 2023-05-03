#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_matrix
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# In Rviz, we generally use Z-up frame as opposed to Z-front as used by the camera. Hence this transformation
TRANSFORM_Z_UP = [[ 0.,  0.,  1., 0.],
				  [-1.,  0.,  0., 0.],
				  [ 0., -1.,  0., 0.],
				  [ 0.,  0.,  0., 1.]]

# The image needs to be flipped left-right and up-down as camera is installed incorrectly on the turtlebot. Hence this transformation
TRANSFORM_FLIP = [[ -1.,  0.,  0., 0.],
				  [  0., -1.,  0., 0.],
				  [  0.,  0.,  1., 0.],
				  [  0.,  0.,  0., 1.]]

RES_SCALE_FACTOR = 1

class TagTracker:
	def __init__(self):
		self.img_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)
		self.tf_pub = tf2_ros.TransformBroadcaster()
		self.str_pub = rospy.Publisher('/apriltag_detections', String, queue_size=10)
		self.marker_pub = rospy.Publisher('point_marker', Marker, queue_size=10)
		self.marker = Marker()
		self.marker.header.frame_id = "map"
		self.marker.ns = "apriltag"
		self.marker.id = 0
		self.marker.type = Marker.POINTS
		self.marker.action = Marker.ADD
		self.marker.pose.orientation.w = 1.0
		self.marker.scale.x = 0.1
		self.marker.scale.y = 0.1
		self.marker.scale.z = 0.1
		self.marker.color.a = 1.0
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0
		# self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
		# tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.tf_listener = tf.TransformListener()
		self.dict_markers = {}

	def callback(self, data):
		#tag_ids = ""
		for tag in data.detections:
			tag_id, transformation_homo, rotation_quaternion = self.getModifiedTransform(tag)
			self.publishApriltagTF(tag.pose.header.stamp, tag_id, transformation_homo, rotation_quaternion)

			try:
				apriltag_wrt_map = self.tf_listener.lookupTransform('/map', f'/tag_{tag_id}', rospy.Time(0))
			except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			
			self.dict_markers[tag_id] = apriltag_wrt_map
			print(self.dict_markers)				

	def publishApriltagTF(self, time_stamp, tag_id, transf_homo, rot_quat):
		# Convert detection pose to a TransformStamped message
		transform = TransformStamped()
		transform.header.stamp = time_stamp
		transform.header.frame_id = 'camera_frame'
		transform.child_frame_id = 'tag_{}'.format(tag_id)

		transform.transform.translation.x = transf_homo[0,3]
		transform.transform.translation.y = transf_homo[1,3]
		transform.transform.translation.z = transf_homo[2,3]

		transform.transform.rotation.x = rot_quat[0]
		transform.transform.rotation.y = rot_quat[1]
		transform.transform.rotation.z = rot_quat[2]
		transform.transform.rotation.w = rot_quat[3]

		# Publish the transform on the tf topic
		self.tf_pub.sendTransform(transform)

	def getModifiedTransform(self, tag):
		tag_id = tag.id[0]

		translation_unscaled_x = tag.pose.pose.pose.position.x
		translation_unscaled_y = tag.pose.pose.pose.position.y
		translation_unscaled_z = tag.pose.pose.pose.position.z
		translation_unscaled = np.array([[translation_unscaled_x],
										 [translation_unscaled_y], 
										 [translation_unscaled_z]])
		
		rotation_quaternion_x = tag.pose.pose.pose.orientation.x
		rotation_quaternion_y = tag.pose.pose.pose.orientation.y
		rotation_quaternion_z = tag.pose.pose.pose.orientation.z
		rotation_quaternion_w = tag.pose.pose.pose.orientation.w
		rotation_quaternion = np.array([rotation_quaternion_x, 
										rotation_quaternion_y, 
										rotation_quaternion_z, 
										rotation_quaternion_w])
		
		rotation_matrix = R.from_quat(rotation_quaternion).as_matrix()
		translation = translation_unscaled/RES_SCALE_FACTOR

		transformation_homo = np.array(TRANSFORM_Z_UP) @ np.array(TRANSFORM_FLIP) @ np.vstack((np.hstack((rotation_matrix, translation)), np.array([0, 0, 0, 1])))
		rotation_quaternion = quaternion_from_matrix(transformation_homo)

		return tag_id, transformation_homo, rotation_quaternion

	def publishMarkers(self, markers_to_publish):
		self.marker.points = markers_to_publish
		self.marker_pub.publish(self.marker)
	
if __name__ == "__main__":
	rospy.init_node('apriltag_detector', anonymous=True)
	tag_tracker = TagTracker()
	
	while not rospy.is_shutdown():
		markers_to_publish = []
		for tag in tag_tracker.dict_markers.keys():
			markers_to_publish.append(Point(tag_tracker.dict_markers[tag][0],
											tag_tracker.dict_markers[tag][1],
											tag_tracker.dict_markers[tag][2]))

		if markers_to_publish:
			print(markers_to_publish)
			tag_tracker.publishMarkers(markers_to_publish)
		
		rospy.Rate(10).sleep()
