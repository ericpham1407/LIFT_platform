#!/usr/bin/env python3
"""
Modified from py_arduino_interface_node.py in barc_hardware_interface. 
"""

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

import copy

# from barc_hardware_interface.barc_interface import BarcArduinoInterface, BarcArduinoInterfaceConfig

# from mpclab_common.pytypes import NodeParamTemplate, VehicleActuation, VehicleState
from luci_msgs.msg import VehicleActuation, VehicleState
# from mpclab_common.mpclab_base_nodes import MPClabNode
from luci_core.base_node import BaseNode, NodeParamTemplate


# from mpclab_msgs.msg import VehicleStateMsg, VehicleActuationMsg, DriveStateMsg


from serial import Serial
from serial.tools import list_ports
import numpy as np
from dataclasses import dataclass, field

# from mpclab_common.pytypes import VehicleState, PythonMsg


"""Copied from barc_interface.py in barc_hardware_interface: START"""
@dataclass
class BarcArduinoInterfaceConfig(NodeParamTemplate):
    device_name: str = field(default = 'Nano')
    port: str   = field(default = None) # default to autoscan
    baud: int   = field(default = 115200)
    dt:   float = field(default = 0.01)
    require_echo: bool = field(default = False)

    # steering_max: int = field(default = 1990)
    # steering_min: int = field(default = 1010)
    # steering_off: int = field(default = 1500)

    # throttle_max: int = field(default = 1900)
    # throttle_min: int = field(default = 1100)
    # throttle_off: int = field(default = 1500)

    # steering_map_mode: str      = field(default='affine')
    # steering_map_params: list   = field(default=None)
    # throttle_map_mode: str      = field(default='affine')
    # throttle_map_params: list   = field(default=None)
    
    # control_mode: str = field(default = 'torque')

class BarcArduinoInterface():

    def __init__(self, config: BarcArduinoInterfaceConfig = BarcArduinoInterfaceConfig(), 
                        print_method=print):
        self.config = config
        self.dt = config.dt
        self.v = 0

        self.print_method = print_method

        self.start()

        return

    def start(self):
        if self.config.port is None:
            self.config.port = self.scan_ports()

            if self.config.port is None:
                raise NotImplementedError('No Arduino Found')
                return

        self.serial = Serial(port         = self.config.port,
                             baudrate     = self.config.baud,
                             timeout      = self.config.dt,
                             writeTimeout = self.config.dt)

        return

    def scan_ports(self):
        ports = list(list_ports.comports())
        for p in ports:
            if self.config.device_name in p.description:
                return p.device

        return None

    def step(self, state:VehicleState):
        return self.write_output(state)

    def write_output(self, x: VehicleState):
        if self.config.control_mode == 'torque':
            steering = self.angle_to_pwm(x.u.u_steer)
            if self.config.throttle_map_mode == 'integration':
                self.v += self.dt*x.u.u_a
                throttle = self.v_to_pwm(self.v, steering)
            elif self.config.throttle_map_mode == 'affine':
                throttle = self.a_to_pwm(x.u.u_a)
            else:
                raise(ValueError("Throttle map mode must be 'affine', 'integration'"))
        elif self.config.control_mode == 'velocity':
            steering = self.angle_to_pwm(x.u.u_steer)
            throttle = self.v_to_pwm(x.u.u_a, steering)
        elif self.config.control_mode == 'direct':
            throttle = x.u.u_a
            steering = x.u.u_steer
        else:
            raise(ValueError("Control mode must be 'torque', 'velocity', or 'direct'"))
        #self.print_method(str(throttle))
        if throttle > 1.2*self.config.throttle_max or throttle < 0.8*self.config.throttle_min:
            self.print_method(f'Throttle limits exceeded, setting to {self.config.throttle_off}')
            throttle = self.config.throttle_off
        if steering > 1.2*self.config.steering_max or steering < 0.8*self.config.steering_min:
            steering = self.config.steering_off
        throttle = int(max(min(throttle, self.config.throttle_max), self.config.throttle_min))
        steering = int(max(min(steering, self.config.steering_max), self.config.steering_min))
        
        try:
            self.serial.flushOutput()
            self.serial.write(b'A0%03d\n'%(throttle - 1000))
            self.serial.write(b'A1%03d\n'%(steering - 1000))
        except Exception as e:
            self.print_method(f'In write_output: {str(e)}')

        return throttle, steering
    
    def write_raw_commands(self, throttle: int, steering: int):
        throttle = max(min(throttle, self.config.throttle_max), self.config.throttle_min)
        steering = max(min(steering, self.config.steering_max), self.config.steering_min)
        
        self.serial.flushOutput()
        self.serial.write(b'A0%03d\n'%(throttle - 1000))
        self.serial.write(b'A1%03d\n'%(steering - 1000))
    
    def write_raw_serial(self, msg: str):
        self.serial.write(msg.encode('ascii'))
        
    def reset_output(self):
        self.serial.flushOutput()
        self.serial.write(b'A0%03d\n'%(self.config.throttle_off-1000))
        self.serial.write(b'A1%03d\n'%(self.config.steering_off-1000))
        return

    def enable_output(self):
        self.serial.write(b'AA000\n')
        return

    def disable_output(self):
        self.serial.write(b'AB000\n')
        return
    
    def read_accel(self):
        try:
            self.serial.flushOutput()
            self.serial.flushInput()
            self.serial.write(b'B2000\n')
            
            msg = self.serial.read_until(expected='\r\n'.encode('ascii'), size=50).decode('ascii')
            ax_start = msg.find('x')
            ay_start = msg.find('y')
            az_start = msg.find('z')
            if ax_start < 0 or ay_start < 0 or az_start < 0:
                return None
            ax = float(msg[ax_start+1:ay_start])*9.81
            ay = float(msg[ay_start+1:az_start])*9.81
            az = float(msg[az_start+1:])*9.81
            return ax, ay, az
        except Exception as e:
            self.print_method(f'In read_accel: {str(e)}')
            return None

    def read_gyro(self):
        try:
            self.serial.flushOutput()
            self.serial.flushInput()
            self.serial.write(b'B3000\n')
            
            msg = self.serial.read_until(expected='\r\n'.encode('ascii'), size=50).decode('ascii')
            wx_start = msg.find('x')
            wy_start = msg.find('y')
            wz_start = msg.find('z')
            if wx_start < 0 or wy_start < 0 or wz_start < 0:
                return None
            wx = float(msg[wx_start+1:wy_start])
            wy = float(msg[wy_start+1:wz_start])
            wz = float(msg[wz_start+1:])
            return wx, wy, wz
        except Exception as e:
            self.print_method(f'In read_gyro: {str(e)}')
            return None

    def read_imu(self):
        try:
            # self.serial.flushOutput()
            # self.serial.flushInput()
            # self.serial.write(b'B4000\n')
            
            msg = self.serial.read_until(expected='\r\n'.encode('ascii'), size=100).decode('ascii')
            # self.print_method(msg)
            a_start = msg.find('a')
            w_start = msg.find('w')
            if a_start < 0 or w_start < 0:
                return None, None
            a_msg = msg[a_start+1:w_start]
            w_msg = msg[w_start+1:]

            ax_start = a_msg.find('x')
            ay_start = a_msg.find('y')
            az_start = a_msg.find('z')
            wx_start = w_msg.find('x')
            wy_start = w_msg.find('y')
            wz_start = w_msg.find('z')
            if ax_start < 0 or ay_start < 0 or az_start < 0:
                return None, None
            if wx_start < 0 or wy_start < 0 or wz_start < 0:
                return None, None
            
            try:
                ax = float(a_msg[ax_start+1:ay_start])*9.81
                ay = float(a_msg[ay_start+1:az_start])*9.81
                az = float(a_msg[az_start+1:])*9.81
                a = [ax, ay, az]
                wx = float(w_msg[wx_start+1:wy_start])
                wy = float(w_msg[wy_start+1:wz_start])
                wz = float(w_msg[wz_start+1:])
                w = [wx, wy, wz]
            except:
                return None, None
        
            return a, w
        except Exception as e:
            self.print_method(f'In read_imu: {str(e)}')
            return None, None
    
    def read_encoders_velocity(self):
        self.serial.flushOutput()
        self.serial.flushInput()
        self.serial.write(b'B5000\n')
        
        msg = self.serial.read_until(expected='\r\n'.encode('ascii'), size=100).decode('ascii')
        fl_start = msg.find('a')
        fr_start = msg.find('b')
        rl_start = msg.find('c')
        rr_start = msg.find('d')
        if fl_start < 0 or fr_start < 0 or rl_start < 0 or rr_start < 0:
            return None
        v_fl = float(msg[fl_start+1:fr_start])
        v_fr = float(msg[fr_start+1:rl_start])
        v_rl = float(msg[rl_start+1:rr_start])
        v_rr = float(msg[rr_start+1:])
        return v_fl, v_fr, v_rl, v_rr

    def read_encoders_count(self):
        self.serial.flushOutput()
        self.serial.flushInput()
        self.serial.write(b'B6000\n')
        
        msg = self.serial.read_until(expected='\r\n'.encode('ascii'), size=100).decode('ascii')
        fl_start = msg.find('a')
        fr_start = msg.find('b')
        rl_start = msg.find('c')
        rr_start = msg.find('d')
        if fl_start < 0 or fr_start < 0 or rl_start < 0 or rr_start < 0:
            return None
        c_fl = int(float.fromhex(msg[fl_start+1:fr_start]))
        c_fr = int(float.fromhex(msg[fr_start+1:rl_start]))
        c_rl = int(float.fromhex(msg[rl_start+1:rr_start]))
        c_rr = int(float.fromhex(msg[rr_start+1:]))
        return c_fl, c_fr, c_rl, c_rr

    def angle_to_pwm(self, steering_angle):
        if self.config.steering_map_mode == 'affine':
            offset, gain = self.config.steering_map_params
            steer_pwm = steering_angle / gain + offset
        elif self.config.steering_map_mode == 'arctan':
            # Popt = [1.01471732e-01, 1.490e+03, -1.57788871e-03, 5.38431760e-01,
            #         1.18338718e-01, 1.37661282e-01]
            # Popt = self.config.steering_map_params
            offset = self.config.steering_map_params[1]
            gain = self.config.steering_map_params[2]
            outer_gain = self.config.steering_map_params[3]
            # lr = Popt[4]
            # lf = Popt[5]
            # L = lr + lf
            steer_pwm = np.tan(steering_angle / outer_gain) / gain + offset
        else:
            raise(ValueError("Steering map mode must be 'affine' or 'arctan'"))
        return steer_pwm
    
    # def v_to_pwm(self, v):
    #     # K = 55.37439384702125
    #     K = self.config.throttle_map_params[0]
    #     throttle_pwm = self.config.throttle_off + K * v
    #     return throttle_pwm

    def v_to_pwm(self, v, steer_pwm):
        Kp, Kn, Lp, Ln = self.config.throttle_map_params
        # throttle_pwm = self.config.throttle_off + (v + L*(steer_pwm-self.config.steering_off)**2)/K
        if v >= 0:
            throttle_pwm = self.config.throttle_off + (v/(Kp-Lp*(steer_pwm-self.config.steering_off)**2))
        else:
            throttle_pwm = self.config.throttle_off + (v/(Kn-Ln*(steer_pwm-self.config.steering_off)**2))
        return throttle_pwm

    def a_to_pwm(self, a):
        gain = self.config.throttle_map_params[0]
        throttle_pwm = a / gain + self.config.throttle_off
        return throttle_pwm

"""Copied from barc_interface.py in barc_hardware_interface: END"""


MSG_TIMEOUT_CTRL = 0.3

class InterfaceNodeParams(NodeParamTemplate):
    def __init__(self):
        self.dt                 = 0.01
        self.interface_params   = BarcArduinoInterfaceConfig()
        
class ArduinoInterfaceNode(BaseNode):

    def __init__(self):
        super().__init__('arduino_interface')
        namespace = self.get_namespace()

        param_template = InterfaceNodeParams()
        self.autodeclare_parameters(param_template, namespace, verbose=False)
        self.autoload_parameters(param_template, namespace, verbose=False)

        self.interface_params = BarcArduinoInterfaceConfig()
        self.interface_params.dt = self.dt
        self.get_logger().info(str(self.interface_params))
        self.interface  = BarcArduinoInterface(self.interface_params,
                                                print_method=self.get_logger().info)
        
        # Get handle to ROS clock
        self.clock = self.get_clock()
        self.t_start = self.clock.now().nanoseconds/1E9

        self.update_timer = self.create_timer(self.dt, self.step)

        self.barc_imu_pub = self.create_publisher(
            Imu,
            'imu',
            qos_profile_sensor_data
        )

        self.control_msg_start = False
        self.last_msg_timestamp = self.clock.now().nanoseconds/1E9 - MSG_TIMEOUT_CTRL
        self.output_enabled = False

        self.t_meas = 0.0

        return

        
    def step(self):
        now= self.clock.now()
        t = now.nanoseconds/1E9
        stamp = now.to_msg()
        
        # a = self.interface.read_accel()
        # w = None
        # w = self.interface.read_gyro()
        a, w = self.interface.read_imu()
        
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        if a is not None:
            ax, ay, az = a
            imu_msg.linear_acceleration.x = ay
            imu_msg.linear_acceleration.y = ax
            imu_msg.linear_acceleration.z = az
        if w is not None:
            wx, wy, wz = w
            imu_msg.angular_velocity.x = wy
            imu_msg.angular_velocity.y = wx
            imu_msg.angular_velocity.z = wz
        if a is not None or w is not None:
            self.barc_imu_pub.publish(imu_msg)  


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
