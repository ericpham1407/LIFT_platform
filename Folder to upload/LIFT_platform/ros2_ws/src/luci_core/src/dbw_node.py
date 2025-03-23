#!/usr/bin/env python3
'''
'''
import rclpy
from rclpy.qos import qos_profile_sensor_data

from luci_core.base_node import BaseNode, NodeParamTemplate
from luci_msgs.msg import VehicleActuation as VehicleActuationMsg
from luci_msgs.msg import VehicleState as VehicleStateMsg

from onr_luci.hardware.motor_interface import MotorInterface
from onr_luci.hardware.dbw_interface import VehicleState, MotorAdaptiveController, VehicleActuation

#class DBWNodeConfig(NodeParamTemplate):
 #    '''
  #   configuration and params for the drive-by-wire node
   #  '''
    # def __init__(self):
     #   self.dbw_config = DBWConfig()

class LuciDBWNode(BaseNode):
    '''
    Luci drive-by-wire system node -
    Subscribes to VehicleActuation messages
    Publishes VehicleState messages
    '''
    u_vehicle: VehicleActuationMsg         # input/actuation of the drive-by-wire system of type VehicleActuation
    controller: MotorAdaptiveController

    def __init__(self):
        super().__init__('dbw_node')
        namespace = self.get_namespace()
        # param_template = DBWNodeConfig()
        # self.autodeclare_parameters(param_template, namespace)
        # self.autoload_parameters(param_template, namespace)

        self.vehicle_actuation_sub = self.create_subscription(
            VehicleActuationMsg,
            'vehicle_actuation',
            self._vehicle_actuation_callback,
            qos_profile_sensor_data
        )
        self.u_vehicle = VehicleActuationMsg()

        self.vehicle_state_pub = self.create_publisher(
            VehicleStateMsg,
            'vehicle_state',
            qos_profile_sensor_data
        )
        self.vehicle_state_msg = VehicleStateMsg()
        self.vehicle_state = VehicleState()

        self.motor_interface = MotorInterface()
        self.controller = MotorAdaptiveController(self.motor_interface, self.vehicle_state)
        # self.controller.iface.reset_motors()

        # self.publish_timer = self.create_timer(self.dt, self._read_telemetry)
        # NOTE - the telemetry is read with every actuation, so do we need a separate timer that reads the telemetry every dt?

    def _vehicle_actuation_callback(self, msg:VehicleActuationMsg):
        py_msg = VehicleActuation()

        self.unpack_msg(msg, py_msg)
        self.info(py_msg)
        # print(type(self.vehicle_state))
    
        self.vehicle_state.u_dbw = py_msg

        self.controller.step(self.vehicle_state)        
        # self.controller.iface.reset_motors()

        # pack the updated vehicle state with the info from reading telemtry and then publish the vehicle state
        # print(self.vehicle_state)
        self.populate_msg(self.vehicle_state_msg, self.vehicle_state)
        self.pub_vehicle_state()
        
    def pub_vehicle_state(self):
        ''' publish the state of the vehicle '''
        self.vehicle_state_pub.publish(self.vehicle_state_msg)

    # def _close(self):
    #     # shutdown any processes that need to be shutdown
    #     super()._close()

def main(args = None):
    '''run the node'''
    rclpy.init(args=args)
    node = LuciDBWNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()