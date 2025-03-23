#!/usr/bin/env python3
''' single gps node '''

import rclpy
from rclpy.qos import qos_profile_sensor_data

from luci_core.base_node import BaseNode, NodeParamTemplate
from luci_msgs.msg import GPSFix

from std_msgs.msg import Bool

from onr_luci.geo.utm import UTMOrigin
from onr_luci.hardware.ublox_gps import GPSClient, GPSConfig


class GPSNodeConfig(NodeParamTemplate):
    ''' configuration of the gps node '''
    def __init__(self):
        super().__init__()
        self.utm_origin_lon: float = 0.
        self.utm_origin_lat: float = 0.
        self.gps_config = GPSConfig()


class LuciGPS(BaseNode):
    '''
    LUCI gps node - publishes GPS fixes and whether or not the gps is connected
    Does not yet publish XY info based on UTM origin.
    '''
    utm_origin_lon: float
    utm_origin_lat: float
    gps_config: GPSConfig
    gps: GPSClient
    gps_fix_msg: GPSFix

    def __init__(self):
        super().__init__('gps')
        namespace = self.get_namespace()
        param_template = GPSNodeConfig()
        self.autodeclare_parameters(param_template, namespace)
        self.autoload_parameters(param_template, namespace)

        self.gps = GPSClient(self.gps_config, output_handler = self.get_logger().info)
        self.utm_origin = UTMOrigin(lon = self.utm_origin_lon, lat = self.utm_origin_lat)

        self.gps_fix_pub = self.create_publisher(
            GPSFix,
            'gps_fix',
            qos_profile_sensor_data
        )
        self.gps_fix_msg = GPSFix()

        self.gps_status_pub = self.create_publisher(
            Bool,
            'gps_connected',
            qos_profile_sensor_data
        )
        self.gps_connected_msg = Bool()

        self.update_timer = self.create_timer(.01, self.read_gps)
        self.status_timer = self.create_timer(1, self.pub_status)

    def read_gps(self):
        '''
        read gps data, publish anything newly available
        '''
        msgs = self.gps.read()
        if msgs:
            # publish most recent message
            self.populate_msg(self.gps_fix_msg, msgs[-1])
            self.gps_fix_pub.publish(self.gps_fix_msg)

    def pub_status(self):
        ''' publish the status of the gps '''
        self.gps_connected_msg.data = self.gps.connected()
        self.gps_status_pub.publish(self.gps_connected_msg)

    def _close(self):
        self.gps.shutdown()
        super()._close()


def main(args=None):
    ''' run the node '''
    rclpy.init(args=args)
    node = LuciGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
