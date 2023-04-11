#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# --- Publishers ---
cmd_pub = None
# --- Time parameters ---
MoveStartTime = None
MoveEndTime = None
MoveDuration = rospy.Duration(20.0)
PirouetteDuration = rospy.Duration(10)

def get_command(msg):
    """
    Receive command sent from motion planner.
    Either forward it along to robot, or replace with our own command.
    We wait MoveDuration seconds between pirouettes.
    When MoveDuration seconds have passed, we begin our spin.
    Params must satisfy PirouetteDuration * msg_new.angular.z = 2 pi = 6.28.
    """
    global MoveStartTime, MoveEndTime
    if rospy.Time.now() >= MoveEndTime:
        # replace motion command with a spin.
        msg_new  = Twist()
        msg_new.linear.x = 0
        msg_new.angular.z = 0.6285  # [radians] Rotates in place for PirouetteDuration -> 360 degree rotation (seconds)'
        PirouetteStartTime = rospy.Time.now()
        PirouetteEndTime = PirouetteStartTime + PirouetteDuration
        while rospy.Time.now() <= PirouetteEndTime:
            # perform a full 360deg spin in place.
            print("Rotating to search for tags.")
            cmd_pub.publish(msg_new)
            rospy.sleep(0.1) # prevent bot from being flooded with commands.
        # setup time of next pirouette.
        MoveStartTime = rospy.Time.now()
        MoveEndTime = MoveStartTime + MoveDuration
    else:
        # forward along command from motion planner.
        cmd_pub.publish(msg)


def main():

    global cmd_pub, MoveStartTime, MoveEndTime

    rospy.init_node('cmd_interrupt_node')

    MoveStartTime = rospy.Time.now()
    MoveEndTime = MoveStartTime + MoveDuration

    # create publisher for cmd_vel.
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # subscribe to command sent by motion planner.
    rospy.Subscriber('/cmd_vel_intermediary', Twist, get_command, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
