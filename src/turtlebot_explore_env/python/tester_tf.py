import rospy
from tf import TransformListener

                       
     
tf = TransformListener()
if tf.frameExists("/base_link") and self.tf.frameExists("/map"):
	t = tf.getLatestCommonTime("/base_link", "/map")
	position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
	print(position, quaternion)
