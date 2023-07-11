#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
import argparse
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from geodesy import utm
from geometry_msgs.msg import PointStamped


class TFBaseLinkPublisher():
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns
        self.initialize = True
        self.map_frame = "/map_locus"
        self.utm_frame = "/utm_origin"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    # def callback(self, value):
    #     # rospy.loginfo(value.pose.pose.position)
    #     br = TransformBroadcaster()
        
    #     time = rospy.Time.now()
    #     if self.initialize:
    #         data = value
    #         self.data_backup = data
    #         self.initialize = False
    #     else:
    #         data = self.data_backup
    #     br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
    #                     (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
    #                     time,
    #                     self.robot_ns + "/map_locus",
    #                     self.robot_ns + self.local_origin)


    def run(self):
        
        while not rospy.is_shutdown():
            if self.tf_buffer.can_transform(self.robot_ns + self.utm_frame,self.robot_ns + self.map_frame,rospy.Time(0), rospy.Duration(1.0)):                
                utm_value = (self.tf_buffer.transform(self.generate_geometry_points([0,0,0],self.robot_ns + self.map_frame),self.robot_ns + self.utm_frame))
                print(self.convert_to_lat_lon(utm_value))

    def convert_to_lat_lon(self,utm_value):                
        utm_coord = utm.UTMPoint(utm_value.point.x,utm_value.point.y,utm_value.point.z,zone=29,band="S")
        print(utm_coord)
        return utm_coord.toMsg()
    
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



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Example usage: python sub_odom_publish_tf.py husky4')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args, unknown = parser.parse_known_args()
    rospy.init_node('pub_odom_tf_map', anonymous=True)
    tf_base_link_publisher = TFBaseLinkPublisher(args.robot_name)
    tf_base_link_publisher.run()