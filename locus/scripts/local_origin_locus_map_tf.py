#!/usr/bin/env python
import rospy
from re import search
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import tf2_geometry_msgs
from geodesy import utm
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import PoseStamped,PointStamped, TransformStamped
import argparse

class TFBaseLinkPublisher():
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns
        self.initialize = True        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.local_origin = "/map"
        if search("uav",self.robot_ns):
            self.local_origin = "/gps_origin"
        self.odom_topic = "/odometry/local_filtered"
        if search("uav",self.robot_ns):
            self.odom_topic = "/odometry/odom_main"
        self.utm_topic = "/utm"
        if search("uav",self.robot_ns):
            self.utm_topic = "/utm_origin"
        self.sub = rospy.Subscriber("/"+robot_ns  +  self.odom_topic  , Odometry, self.callback)

    def callback(self, value):
        # rospy.loginfo(value.pose.pose.position)
        time = rospy.Time.now()
        if self.initialize:
            data = value
            self.data_backup = data
            self.initialize = False
        else:
            data = self.data_backup
        try:
            transform_stamped = TransformStamped()
            # Fill in the necessary fields of the TransformStamped message
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = self.robot_ns + self.local_origin
            transform_stamped.child_frame_id = self.robot_ns + "/map_locus"
            transform_stamped.transform.translation.x = data.pose.pose.position.x
            transform_stamped.transform.translation.y = data.pose.pose.position.y
            transform_stamped.transform.translation.z = data.pose.pose.position.z
            transform_stamped.transform.rotation.x = data.pose.pose.orientation.x
            transform_stamped.transform.rotation.y = data.pose.pose.orientation.y
            transform_stamped.transform.rotation.z = data.pose.pose.orientation.z
            transform_stamped.transform.rotation.w = data.pose.pose.orientation.w

            self.tf_broadcaster.sendTransform(transform_stamped)
            print(self.convert_to_lat_lon(self.tf_buffer.transform(self.generate_geometry_points([0,0,0],self.robot_ns + "/map_locus"),"uav1/utm_origin",rospy.Duration(1))))

        except:
            pass
    
    def generate_geometry_points(self,point,frame_id):
        # print(point)
        header = Header()
        header.seq=1
        header.stamp = rospy.Time()
        header.frame_id = frame_id
        geometry_point = PointStamped()
        geometry_point.header = header
        geometry_point.point.x = point[0]
        geometry_point.point.y = point[1]
        geometry_point.point.z = point[2]
        return geometry_point

    def convert_to_lat_lon(self,utm_value):
        utm_coord = utm.UTMPoint(utm_value.point.x,utm_value.point.y,utm_value.point.z,29,"S")
        return utm_coord.toMsg()
    
    def run(self):        
        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Example usage: python sub_odom_publish_tf.py husky4')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args, unknown = parser.parse_known_args()
    rospy.init_node('pub_odom_tf_map', anonymous=True)
    tf_base_link_publisher = TFBaseLinkPublisher(args.robot_name)
    tf_base_link_publisher.run()