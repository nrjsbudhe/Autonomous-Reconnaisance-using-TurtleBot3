import rospy
from tf import TransformListener
from geometry_msgs.msg import TransformStamped

rospy.init_node('tf_listener')

listener = TransformListener()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        trans, rot = listener.lookupTransform('base_link', 'map', rospy.Time(0))
        rospy.loginfo("Transform from base_link to map: Translation: %s, Rotation: %s", trans, rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    rate.sleep()

