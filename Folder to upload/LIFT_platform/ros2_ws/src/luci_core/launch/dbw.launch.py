#!/usr/bin/env python3
''' gps launch file '''
import sys
import traceback

from launch import LaunchDescription

from luci_core.launch_utils import get_node, get_config_directory

def generate_launch_description():
    ''' the launch description '''
    try:
        gps_parameters = [get_config_directory('memorial_glade_geo_config.yaml', 'luci_core')]
        namespace  = ''

        gps_node = get_node('luci_core', 'gps.py',
                            parameters=gps_parameters, namespace=namespace)
        imu_node = get_node('luci_core', 'arduino_node.py',
                            namespace=namespace)
        control_node = get_node('luci_core', 'dbw_node.py',
                                namespace=namespace)

        return LaunchDescription([gps_node, imu_node, control_node])

    except Exception:
        print('There was an error inside your launch code!!!! 🙄️')
        print('ROS 2 won\'t handle it nicely, so here is the real error:')
        print('')
        traceback.print_exc()

        sys.exit(0)
