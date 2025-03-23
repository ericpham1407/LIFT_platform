#!/usr/bin/env python3
'''
'''
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data

from xbox360controller import Xbox360Controller

from luci_core.base_node import BaseNode, NodeParamTemplate
from luci_msgs.msg import VehicleActuation as VehicleActuationMsg

from onr_luci.hardware.dbw_interface import VehicleControlMode

class JoystickConfig(NodeParamTemplate):
    '''
    joystick node config
    '''
    def __init__(self):
        # super().__init__()
        self.dt = 0.01
        self.velocity_max: float = 1.0
        self.omega_max: float = 5.0
        self.deadzone_threshold = 0.04
        # self.max_joystick = None

class JoystickNode(BaseNode):
    '''
    joystick node
    '''
    dt: float
    velocity_max: float
    omega_max: float

    joystick: Xbox360Controller
    velocity_input: float
    omega_input:float

    def __init__(self):
        super().__init__('joystick_node')
        namespace = self.get_namespace()
        param_template = JoystickConfig()
        self.autodeclare_parameters(param_template, namespace)
        self.autoload_parameters(param_template, namespace)
        
        self.joystick = Xbox360Controller(0, axis_threshold=0.0)

        self.vehicle_actuation_pub = self.create_publisher(
            VehicleActuationMsg,
            'vehicle_actuation',
            qos_profile_sensor_data
        )
        self.vehicle_actuation_msg = VehicleActuationMsg()

        self.get_input_timer = self.create_timer(self.dt, self._get_input)

    def _get_input(self):
        '''
        get input from the joystick
        '''
        # get input from the joystick
        self.velocity_input, self.omega_input = (-self.joystick.axis_l.y, -self.joystick.axis_r.x)
        # self.get_logger().info(f'Joystick Input Raw value: {self.velocity_input:.4f}'
        self.velocity_input = self.velocity_input * self.velocity_max
        self.velocity_input = np.clip(self.velocity_input, -self.velocity_max, self.velocity_max)
        self.omega_input = self.omega_input * self.omega_max
        self.omega_input = np.clip(self.omega_input, -self.omega_max, self.omega_max)
        if abs(self.velocity_input) <= self.deadzone_threshold:
            self.velocity_input = 0.
        if abs(self.omega_input) <= self.deadzone_threshold:
            self.omega_input = 0.

        # NOTE - for now, later figure out how to puclish the mode / where that input is coming from
        self.vehicle_actuation_msg.mode = VehicleControlMode.VELOCITY.value
        self.vehicle_actuation_msg.velocity = self.velocity_input
        self.vehicle_actuation_msg.omega = self.omega_input

        # publish input
        self.vehicle_actuation_pub.publish(self.vehicle_actuation_msg)

def main(args = None):
    '''run the node'''
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
