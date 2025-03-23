#!/usr/bin/env python3
'''
tets for basic node utilities
intended to cover necessary operations for passing configuration and
operation data around

tested features:

configuration:
    primitive types
    nested structures
    Enums
    optional values (typing.Optional)

messages:
    primitive types
    nested structures
    Enums
    limited support for typing.Optional and typing.Union*

*- defaults to non-None-type for Optionals and first type for Unions,
unexpected behaviour may occur
'''
from enum import Enum
from dataclasses import dataclass, field

import rclpy
from rclpy.qos import qos_profile_sensor_data

from luci_core.base_node import BaseNode, NodeParamTemplate
from luci_msgs.msg import GPSFix as GPSFixMsg
from luci_msgs.msg import PoseVel as PoseVelMsg
from luci_msgs.msg import MotorCommand as MotorCommandMsg
from luci_msgs.msg import MotorReadout as MotorReadoutMsg

from onr_luci.pytypes import PoseVel, PythonMsg
from onr_luci.hardware.ublox_gps import GPSConfig, GPSFix
from onr_luci.hardware.motor_interface import MotorReadout, MotorCommand, MotorLocation

class _TestEnum(Enum):
    A = 0
    B = 1
    C = 2

@dataclass
class _TestStruct(PythonMsg):
    test_mode: _TestEnum = field(default = _TestEnum.A)
    test_foo: int = field(default = 0)

class TestNodeConfig(NodeParamTemplate):
    ''' configuration of the gps node '''
    def __init__(self):
        super().__init__()
        self.pub_count: int = 3
        self.utm_origin_lon: float = 0.
        self.utm_origin_lat: float = 0.
        self.gps_config = GPSConfig()
        self.pose_vel = PoseVel()
        self.test_enum = _TestEnum.B
        self.test_struct = _TestStruct()

        #NOTE: fast way to create a YAML file for config is to copy paste output of this:
        # print statements are suppressed when inside of a launch file.
        print('Default YAML config file contents:')
        print(self.spew_yaml())
        print('')


class TestNode(BaseNode):
    '''
    Node for testing basic features
    '''
    utm_origin_lon: float
    utm_origin_lat: float
    gps_config: GPSConfig
    pose_vel: PoseVel
    test_enum: _TestEnum
    test_struct: _TestStruct
    pub_count: int

    def __init__(self):
        super().__init__('test_node')
        namespace = self.get_namespace()
        param_template = TestNodeConfig()
        self.autodeclare_parameters(param_template, namespace, verbose=True)
        self.autoload_parameters(param_template, namespace, verbose=True)

        #self.gps_config.print()
        #self.pose_vel.print()
        #print(self.test_enum)
        #self.test_struct.print()

        self.gps_fix_pub = self.create_publisher(
            GPSFixMsg,
            'gps_fix',
            qos_profile_sensor_data
        )
        self.gps_fix_sub = self.create_subscription(
            GPSFixMsg,
            'gps_fix',
            self._gps_fix_callback,
            qos_profile_sensor_data
        )
        self.gps_fix_msg = GPSFixMsg()
        self.gps_fix = GPSFix()


        self.pose_vel_pub = self.create_publisher(
            PoseVelMsg,
            'pose_vel',
            qos_profile_sensor_data
        )
        self.pose_vel_sub = self.create_subscription(
            PoseVelMsg,
            'pose_vel',
            self._pose_vel_callback,
            qos_profile_sensor_data
        )
        self.pose_vel_msg = PoseVelMsg()
        self.pose_vel = PoseVel()
        self.pose_vel.x.from_vec((1,2,3))
        self.pose_vel.q.from_vec((4,5,6,7))

        self.motor_cmd_pub = self.create_publisher(
            MotorCommandMsg,
            'motor_command',
            qos_profile_sensor_data,
        )
        self.motor_cmd_sub = self.create_subscription(
            MotorCommandMsg,
            'motor_command',
            self._motor_cmd_callback,
            qos_profile_sensor_data
        )
        self.motor_cmd_msg = MotorCommandMsg()
        self.motor_cmd = MotorCommand()
        self.motor_cmd.motor = MotorLocation.FR

        self.motor_readout_pub = self.create_publisher(
            MotorReadoutMsg,
            'motor_readout',
            qos_profile_sensor_data,
        )
        self.motor_readout_sub = self.create_subscription(
            MotorReadoutMsg,
            'motor_readout',
            self._motor_readout_callback,
            qos_profile_sensor_data
        )
        self.motor_readout_msg = MotorReadoutMsg()
        self.motor_readout = MotorReadout()

        self.update_timer = self.create_timer(1, self.pub)
        self.msgs_published = -1

    def pub(self):
        '''
        read gps data, publish anything newly available
        '''
        self.msgs_published += 1
        if self.msgs_published == self.pub_count:
            self._close()

        self.populate_msg(self.gps_fix_msg, self.gps_fix)
        self.gps_fix_pub.publish(self.gps_fix_msg)

        self.populate_msg(self.pose_vel_msg, self.pose_vel)
        self.pose_vel_pub.publish(self.pose_vel_msg)

        self.populate_msg(self.motor_cmd_msg, self.motor_cmd)
        self.motor_cmd_pub.publish(self.motor_cmd_msg)

        self.populate_msg(self.motor_readout_msg, self.motor_readout)
        self.motor_readout_pub.publish(self.motor_readout_msg)


    def _gps_fix_callback(self, msg: GPSFixMsg):
        py_msg = GPSFix()
        self.unpack_msg(msg, py_msg)
        self.info(py_msg)

    def _pose_vel_callback(self, msg: PoseVelMsg):
        py_msg = PoseVel()
        self.unpack_msg(msg, py_msg)
        self.info(py_msg)

    def _motor_cmd_callback(self, msg: MotorCommandMsg):
        py_msg = MotorCommand()
        self.unpack_msg(msg, py_msg)
        self.info(py_msg)

    def _motor_readout_callback(self, msg: MotorReadoutMsg):
        py_msg = MotorReadout()
        self.unpack_msg(msg, py_msg)
        self.info(py_msg)

def main(args=None):
    ''' run the node '''
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
