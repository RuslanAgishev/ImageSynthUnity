#!/usr/bin/env python3

import rospy
import tf2_ros
from tf import TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
from pyquaternion import Quaternion


class OdomPublisher(object):
    """docstring for OdomPublisher"""
    def __init__(self, pose_topic='/pose',
                       odom_frame='/odom',
                       robot_frame='/base_footprint'):
        super(OdomPublisher, self).__init__()
        self.initialized_odom_frame = False
        self.odom_pose = None
        self.odom_orient = None
        self.robot_frame = robot_frame
        self.odom_frame = odom_frame
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tl = TransformListener()
        
    def send_transform(self, pose, orient, frame_id, child_frame):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = orient.x
        t.transform.rotation.y = orient.y
        t.transform.rotation.z = orient.z
        t.transform.rotation.w = orient.w
        self.br.sendTransform(t)

    def callback(self, pose_msg):
        pose = pose_msg.pose.position
        orient = pose_msg.pose.orientation

        if not self.initialized_odom_frame:
            self.initialized_odom_frame = True
            self.odom_pose = pose
            self.odom_orient = Quaternion(x=orient.x,
                                          y=orient.y,
                                          z=orient.z,
                                          w=orient.w).normalised
        else:
            # send robot pose in odom frame
            pose_np = np.array([pose.x - self.odom_pose.x,
                                pose.y - self.odom_pose.y,
                                pose.z - self.odom_pose.z])
            orient = Quaternion(x=orient.x,
                                y=orient.y,
                                z=orient.z,
                                w=orient.w).normalised #* self.odom_orient.inverse

            self.send_transform(pose_np, orient, self.odom_frame, self.robot_frame)

        # send odom pose in world (Unity) frame
        pose_np = np.array([self.odom_pose.x,
                            self.odom_pose.y,
                            self.odom_pose.z])
        orient = self.odom_orient

        self.send_transform(pose_np, orient, pose_msg.header.frame_id, self.odom_frame)

        # self.tl.waitForTransform("Unity", "odom", rospy.Time(0), rospy.Duration(1))
        # position, quaternion = self.tl.lookupTransform("Unity", "odom", rospy.Time(0))
        # print(position, quaternion)


def main_program():
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    rospy.init_node("pose_to_tf_broadcaster")
    odom_pub = OdomPublisher()
    rospy.spin()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass
