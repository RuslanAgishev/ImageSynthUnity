#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped


def callback(pose_msg):
    pose = pose_msg.pose.position
    orient = pose_msg.pose.orientation

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = pose_msg.header.frame_id
    t.child_frame_id = target_frame
    t.transform.translation.x = pose.x
    t.transform.translation.y = pose.y
    t.transform.translation.z = pose.z
    t.transform.rotation.x = orient.x
    t.transform.rotation.y = orient.y
    t.transform.rotation.z = orient.z
    t.transform.rotation.w = orient.w
    br.sendTransform(t)


def main_program(pose_topic="/pose", robot_frame="/base_footprint"):
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    global target_frame
    rospy.init_node("pose_to_tf_broadcaster")
    target_frame = robot_frame
    pose_sub = rospy.Subscriber(pose_topic, PoseStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass
