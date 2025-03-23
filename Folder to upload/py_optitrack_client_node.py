#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

import sys, datetime

from optitrack_client.NatNetClient import NatNetClient
from optitrack_client.VelocityFilter import VelocityFilter

class OptiTrackNode(Node):
    def __init__(self):
    
        super().__init__('optitrack_node')
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('multicast_group', '239.255.42.99')
        self.declare_parameter('multicast_port', 1511)
        self.declare_parameter('body_id', 1)
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('pub_accel', False)
        self.declare_parameter('lin_accel_topic', '')
        self.declare_parameter('ang_accel_topic', '')
        # self.declare_parameter('link_name', 'odom')
        # self.declare_parameter('child_link_name', 'tracker_link')

        (dt, multicast_group, multicast_port, body_id, odom_topic, pub_accel, lin_accel_topic, ang_accel_topic) = self.get_parameters(
            ['dt', 'multicast_group', 'multicast_port', 'body_id', 'odom_topic', 'pub_accel', 'lin_accel_topic', 'ang_accel_topic'])
        
        self.dt = dt.get_parameter_value().double_value
        self.multicast_group = multicast_group.get_parameter_value().string_value
        self.multicast_port = multicast_port.get_parameter_value().integer_value
        self.body_id = body_id.get_parameter_value().integer_value
        self.odom_topic = odom_topic.get_parameter_value().string_value
        self.pub_accel = pub_accel.get_parameter_value().bool_value
        self.lin_accel_topic = lin_accel_topic.get_parameter_value().string_value
        self.ang_accel_topic = ang_accel_topic.get_parameter_value().string_value

        odom_topic_name = '/optitrack/odom' if self.odom_topic == "" else self.odom_topic
        self.odom_pub = self.create_publisher(Odometry, odom_topic_name,
            qos_profile=qos_profile_sensor_data)

        if self.pub_accel:
            lin_accel_topic_name = '/optitrack/lin_accel' if self.lin_accel_topic == "" else self.lin_accel_topic
            self.lin_accel_pub = self.create_publisher(Vector3Stamped, lin_accel_topic_name,
                qos_profile=qos_profile_sensor_data)

            ang_accel_topic_name = '/optitrack/ang_accel' if self.ang_accel_topic == "" else self.ang_accel_topic
            self.ang_accel_pub = self.create_publisher(Vector3Stamped, ang_accel_topic_name,
                qos_profile=qos_profile_sensor_data)

        self.streaming_client = NatNetClient()
        self.streaming_client.set_use_multicast(True)
        self.streaming_client.set_print_level(0)

        self.velocity_filter = VelocityFilter(self.streaming_client.mocap_data_queue, 
                                                body_id=self.body_id,
                                                print_method=self.get_logger().info)

        self.streaming_client.run()
        self.velocity_filter.run()

        self.clock = self.get_clock()
        
        self.update_timer = self.create_timer(self.dt, self.step)

    
    def step(self):
        if not rclpy.ok():
            sys.exit(0)
    
        measurement_data = self.velocity_filter.get_measurement_data()
        if measurement_data:
            stamp = self.get_clock().now().to_msg()
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            # odom_msg.header.frame_id = self.link_name

            # odom_msg.child_frame_id = self.child_link_name

            odom_msg.pose.pose.position.x = measurement_data['x'][0]
            odom_msg.pose.pose.position.y = measurement_data['x'][1]
            odom_msg.pose.pose.position.z = measurement_data['x'][2]

            odom_msg.pose.pose.orientation.x = measurement_data['q'][0]
            odom_msg.pose.pose.orientation.y = measurement_data['q'][1]
            odom_msg.pose.pose.orientation.z = measurement_data['q'][2]
            odom_msg.pose.pose.orientation.w = measurement_data['q'][3]

            odom_msg.twist.twist.linear.x = measurement_data['vb'][0]
            odom_msg.twist.twist.linear.y = measurement_data['vb'][1]
            odom_msg.twist.twist.linear.z = measurement_data['vb'][2]

            odom_msg.twist.twist.angular.x = measurement_data['w'][0]
            odom_msg.twist.twist.angular.y = measurement_data['w'][1]
            odom_msg.twist.twist.angular.z = measurement_data['w'][2]

            self.odom_pub.publish(odom_msg)

            if self.pub_accel:
                lin_accel_msg = Vector3Stamped()
                lin_accel_msg.header.stamp = stamp

                lin_accel_msg.vector.x = measurement_data['ab'][0]
                lin_accel_msg.vector.y = measurement_data['ab'][1]
                lin_accel_msg.vector.z = measurement_data['ab'][2]

                self.lin_accel_pub.publish(lin_accel_msg)

                ang_accel_msg = Vector3Stamped()
                ang_accel_msg.header.stamp = stamp

                ang_accel_msg.vector.x = measurement_data['aa'][0]
                ang_accel_msg.vector.y = measurement_data['aa'][1]
                ang_accel_msg.vector.z = measurement_data['aa'][2]

                self.ang_accel_pub.publish(ang_accel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
