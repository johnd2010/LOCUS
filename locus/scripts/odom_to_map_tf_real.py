#!/usr/bin/env python
import rospy, tf
from re import search
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import tf_conversions
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import PoseStamped
import argparse
import numpy as np
import math

class TFBaseLinkPublisher():
    def __init__(self, robot_ns=""):
        self.robot_ns = robot_ns
        self.frame_base_link = "base_link"
        self.frame_map = "map"
        self.frame_odom = "odom"

        self.sub = rospy.Subscriber(    "/locus/odometry" , Odometry, self.callback)
        self.path = Path()
        self.path_pub = rospy.Publisher( "/path", Path, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.odom = None

    def callback(self, data):
        rospy.loginfo(data.pose.pose.position)
        self.odom = data
        


    def get_map_to_odom(self,data):
        time = rospy.Time.now()
        tfpose_map_base = data
        br = TransformBroadcaster()
        rot_mb = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
        # Map frame -> base_link (real pose) in map frame
        #tfpose_map_base, rot_mb = self.get_new_tf_data(self.tf_listener, self.frame_map, self.frame_robot)
        theta_mb = \
            tf.transformations.euler_from_quaternion(quaternion=(rot_mb[0], rot_mb[1], rot_mb[2], rot_mb[3]))[2]

        # Odometry frame -> base_link in odom frame
        tfpose_odom_base, rot_ob = self.get_new_tf_data(self.tf_listener, self.frame_odom, self.frame_base_link)
        theta_ob = \
            tf.transformations.euler_from_quaternion(quaternion=(rot_ob[0], rot_ob[1], rot_ob[2], rot_ob[3]))[2]

        # Map frame -> odom in map frame
        theta_mo = theta_mb - theta_ob

        # using homogeneous coordinates:
        # m_H_r = Robot (husky) in Map frame:
        m_x_r = tfpose_map_base.pose.position.x
        m_y_r = tfpose_map_base.pose.position.y
        m_theta_r = theta_mb
        m_H_r = np.array([[math.cos(m_theta_r), -math.sin(m_theta_r), 0, m_x_r],
                            [math.sin(m_theta_r), math.cos(m_theta_r), 0, m_y_r],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        # o_H_b = Base_link in Odom frame:
        o_x_b = tfpose_odom_base.pose.position.x
        o_y_b = tfpose_odom_base.pose.position.y
        o_theta_b = theta_ob
        o_H_b = np.array([[math.cos(o_theta_b), -math.sin(o_theta_b), 0, o_x_b],
                            [math.sin(o_theta_b), math.cos(o_theta_b), 0, o_y_b],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        m_H_o = np.matmul(m_H_r, np.linalg.inv(o_H_b))
        m_x_o = m_H_o[0][-1]
        m_y_o = m_H_o[1][-1]

        tfpose_map_odom = PoseStamped()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta_mo)
        tfpose_map_odom.pose.orientation.x = quaternion[0]
        tfpose_map_odom.pose.orientation.y = quaternion[1]
        tfpose_map_odom.pose.orientation.z = quaternion[2]
        tfpose_map_odom.pose.orientation.w = quaternion[3]
        tfpose_map_odom.pose.position.x = m_x_o  # looks right but needs double check
        tfpose_map_odom.pose.position.y = m_y_o

        br.sendTransform((tfpose_map_odom.pose.position.x, tfpose_map_odom.pose.position.y, tfpose_map_odom.pose.position.z),
                        (tfpose_map_odom.pose.orientation.x, tfpose_map_odom.pose.orientation.y, tfpose_map_odom.pose.orientation.z, tfpose_map_odom.pose.orientation.w),
                        time,
                        self.frame_odom,
                        self.frame_map)

        return tfpose_map_odom
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
            if self.odom is not None:   
                self.get_map_to_odom(self.odom.pose)
    
    def generate_pose_from_tf(self, trans, rot):

        newest_pose = PoseStamped()
        newest_pose.pose.position.x = trans[0]
        newest_pose.pose.position.y = trans[1]
        newest_pose.pose.position.z = trans[2]

        newest_pose.pose.orientation.x = rot[0]
        newest_pose.pose.orientation.y = rot[1]
        newest_pose.pose.orientation.z = rot[2]
        newest_pose.pose.orientation.w = rot[3]

        return newest_pose


    def get_new_tf_data(self, tf_listener, parent, child):
        # try:
        # rospy.logwarn(parent)
        # rospy.logwarn(child)
        (trans, rot) = tf_listener.lookupTransform(parent, child, rospy.Time(0))
        pose = self.generate_pose_from_tf(trans, rot)
        self.tf_exists = True
        return pose, rot

        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
        #     rospy.logerr_throttle(10000, "Error obtaining requested TFs - if this error persists, this message will be "
        #                           "printed every 10 seconds")
        #     tf_message = "Transform between " + self.frame_gazebo + " and " + self.frame_robot + " does not exist yet"
        #     rospy.logerr_throttle(10, tf_message)
        #     self.tf_exists = False
        #     raise tf.Exception


if __name__ == '__main__':
    rospy.init_node('pub_odom_tf_map', anonymous=True)
    tf_base_link_publisher = TFBaseLinkPublisher()
    tf_base_link_publisher.run()